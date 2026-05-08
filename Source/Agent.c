// VolMirrorAgent — userland companion to VolMirror.driver.
//
// Enumerates real audio outputs, selects those with locked hardware volume
// (e.g. HDMI, DisplayPort), and assigns each one to a virtual VolMirror
// "(Vol)" mirror device by writing the driver's `'flwt'`/`'fldn'` custom
// properties. For each active mirror, runs an in-process IO bridge:
//
//   App → virtual device output stream
//   driver applies volume → driver ring
//   agent IOProc on virtual device input stream → agent ring
//   agent IOProc on real device output stream  → real hardware
//
// Background-only. No UI. dispatch_main(); SIGTERM-clean exit.

#include <CoreAudio/AudioHardware.h>
#include <CoreFoundation/CoreFoundation.h>
#include <dispatch/dispatch.h>
#include <os/log.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VMLOG(fmt, ...) os_log(OS_LOG_DEFAULT, "[VolMirrorAgent] " fmt, ##__VA_ARGS__)
#define VMERR(fmt, ...) os_log_error(OS_LOG_DEFAULT, "[VolMirrorAgent] " fmt, ##__VA_ARGS__)

// Custom property selectors — must match Source/Plugin.c.
#define kVMProperty_FollowTarget    'flwt'
#define kVMProperty_FollowName      'fldn'

#define kMaxBridges        8
#define kAgentRingFrames   8192     // power of 2; ~170 ms at 48 kHz of slack between IOProcs

// SPSC ring (stereo float32 interleaved).
typedef struct {
    float*       data;
    UInt32       capacity;
    atomic_uint  head;
    atomic_uint  tail;
} Ring;

static void ring_init(Ring* r, UInt32 cap) {
    r->capacity = cap;
    r->data = (float*)calloc((size_t)cap * 2, sizeof(float));
    atomic_store(&r->head, 0);
    atomic_store(&r->tail, 0);
}
static void ring_reset(Ring* r) {
    atomic_store(&r->head, 0);
    atomic_store(&r->tail, 0);
}

// Producer: write `frames` of stereo float32. Drops excess when full.
static UInt32 ring_write(Ring* r, const float* src, UInt32 frames) {
    if (!r->data) return 0;
    UInt32 mask = r->capacity - 1;
    UInt32 head = atomic_load_explicit(&r->head, memory_order_acquire);
    UInt32 tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    UInt32 used = (tail - head) & mask;
    UInt32 freeSpace = r->capacity - 1 - used;
    UInt32 toWrite = frames < freeSpace ? frames : freeSpace;
    for (UInt32 i = 0; i < toWrite; i++) {
        UInt32 idx = (tail + i) & mask;
        r->data[idx*2 + 0] = src[i*2 + 0];
        r->data[idx*2 + 1] = src[i*2 + 1];
    }
    atomic_store_explicit(&r->tail, (tail + toWrite) & mask, memory_order_release);
    return toWrite;
}

// Consumer: read up to `frames` into stereo dst (channelStride frames). Underrun → silence.
static UInt32 ring_read(Ring* r, float* dst, UInt32 channelStride, UInt32 frames) {
    if (!r->data) {
        for (UInt32 i = 0; i < frames; i++) {
            dst[i*channelStride + 0] = 0.0f; dst[i*channelStride + 1] = 0.0f;
        }
        return 0;
    }
    UInt32 mask = r->capacity - 1;
    UInt32 head = atomic_load_explicit(&r->head, memory_order_relaxed);
    UInt32 tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    UInt32 used = (tail - head) & mask;
    UInt32 toRead = frames < used ? frames : used;
    for (UInt32 i = 0; i < toRead; i++) {
        UInt32 idx = (head + i) & mask;
        dst[i*channelStride + 0] = r->data[idx*2 + 0];
        dst[i*channelStride + 1] = r->data[idx*2 + 1];
    }
    for (UInt32 i = toRead; i < frames; i++) {
        dst[i*channelStride + 0] = 0.0f; dst[i*channelStride + 1] = 0.0f;
    }
    atomic_store_explicit(&r->head, (head + toRead) & mask, memory_order_release);
    return toRead;
}

// Active per-(virtDev, realDev) bridge.
typedef struct {
    AudioObjectID        virtDev;
    AudioObjectID        realDev;
    char                 realUID[256];
    char                 virtUID[256];   // "VolMirror/<realUID>", for diff comparisons
    AudioDeviceIOProcID  virtProc;       // input on virtDev → ring
    AudioDeviceIOProcID  realProc;       // ring → output on realDev
    Ring                 ring;
    bool                 active;
} Bridge;

static Bridge          gBridges[kMaxBridges];
static dispatch_queue_t gReconcileQ;
static dispatch_source_t gDebounce;
static atomic_int       gShutdown = 0;

// ---------- HAL property helpers ----------

static OSStatus get_cfstring(AudioObjectID dev, AudioObjectPropertySelector sel,
                             AudioObjectPropertyScope scope, char* dst, size_t dstSize) {
    AudioObjectPropertyAddress a = { sel, scope, kAudioObjectPropertyElementMain };
    CFStringRef s = NULL; UInt32 sz = sizeof(s);
    OSStatus err = AudioObjectGetPropertyData(dev, &a, 0, NULL, &sz, &s);
    if (err != noErr) { dst[0] = 0; return err; }
    if (!s) { dst[0] = 0; return -1; }
    if (!CFStringGetCString(s, dst, (CFIndex)dstSize, kCFStringEncodingUTF8)) dst[0] = 0;
    CFRelease(s);
    return noErr;
}

static OSStatus set_cfstring(AudioObjectID dev, AudioObjectPropertySelector sel,
                             const char* value) {
    AudioObjectPropertyAddress a = { sel, kAudioObjectPropertyScopeGlobal,
                                     kAudioObjectPropertyElementMain };
    CFStringRef s = CFStringCreateWithCString(NULL, value ? value : "", kCFStringEncodingUTF8);
    if (!s) return -1;
    OSStatus err = AudioObjectSetPropertyData(dev, &a, 0, NULL, sizeof(s), &s);
    CFRelease(s);
    return err;
}

// ---------- Real-device classification ----------

static bool real_is_output_with_locked_volume(AudioObjectID dev) {
    char uid[256] = {0};
    get_cfstring(dev, kAudioDevicePropertyDeviceUID, kAudioObjectPropertyScopeGlobal,
                 uid, sizeof(uid));
    if (strncmp(uid, "VolMirror/", 10) == 0) return false;     // never mirror ourselves

    AudioObjectPropertyAddress streams = { kAudioDevicePropertyStreams,
                                           kAudioObjectPropertyScopeOutput,
                                           kAudioObjectPropertyElementMain };
    UInt32 sz = 0;
    if (AudioObjectGetPropertyDataSize(dev, &streams, 0, NULL, &sz) != noErr) return false;
    if (sz == 0) return false;                                  // no output streams

    AudioObjectPropertyAddress vol = { kAudioDevicePropertyVolumeScalar,
                                       kAudioObjectPropertyScopeOutput,
                                       kAudioObjectPropertyElementMain };
    Boolean settable = false;
    if (AudioObjectIsPropertySettable(dev, &vol, &settable) == noErr && settable) return false;
    return true;                                                // missing/unsettable = locked
}

// ---------- Device enumeration ----------

static AudioObjectID* enumerate_devices(UInt32* outN) {
    AudioObjectPropertyAddress a = { kAudioHardwarePropertyDevices,
                                     kAudioObjectPropertyScopeGlobal,
                                     kAudioObjectPropertyElementMain };
    UInt32 sz = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &a, 0, NULL, &sz) != noErr) {
        *outN = 0; return NULL;
    }
    AudioObjectID* devs = (AudioObjectID*)malloc(sz);
    if (!devs) { *outN = 0; return NULL; }
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &a, 0, NULL, &sz, devs) != noErr) {
        free(devs); *outN = 0; return NULL;
    }
    *outN = sz / sizeof(AudioObjectID);
    return devs;
}

// ---------- IO bridge ----------

static OSStatus virt_in_io_proc(AudioObjectID inDevice,
                                const AudioTimeStamp* inNow,
                                const AudioBufferList* inInputData,
                                const AudioTimeStamp* inInputTime,
                                AudioBufferList* outOutputData,
                                const AudioTimeStamp* inOutputTime,
                                void* clientData) {
    (void)inDevice; (void)inNow; (void)inInputTime; (void)outOutputData; (void)inOutputTime;
    Bridge* b = (Bridge*)clientData;
    if (!b || !inInputData || inInputData->mNumberBuffers == 0) return noErr;
    const AudioBuffer* buf = &inInputData->mBuffers[0];
    if (buf->mNumberChannels != 2 || !buf->mData) return noErr;
    UInt32 frames = buf->mDataByteSize / (sizeof(float) * 2);
    ring_write(&b->ring, (const float*)buf->mData, frames);
    return noErr;
}

static OSStatus real_out_io_proc(AudioObjectID inDevice,
                                 const AudioTimeStamp* inNow,
                                 const AudioBufferList* inInputData,
                                 const AudioTimeStamp* inInputTime,
                                 AudioBufferList* outOutputData,
                                 const AudioTimeStamp* inOutputTime,
                                 void* clientData) {
    (void)inDevice; (void)inNow; (void)inInputData; (void)inInputTime; (void)inOutputTime;
    Bridge* b = (Bridge*)clientData;
    if (!b || !outOutputData || outOutputData->mNumberBuffers == 0) return noErr;
    AudioBuffer* buf = &outOutputData->mBuffers[0];
    UInt32 channels = buf->mNumberChannels;
    if (channels < 2 || !buf->mData) return noErr;
    UInt32 frames = buf->mDataByteSize / (sizeof(float) * channels);
    float* out = (float*)buf->mData;
    // Zero everything (covers extra channels too).
    memset(out, 0, frames * sizeof(float) * channels);
    ring_read(&b->ring, out, channels, frames);
    return noErr;
}

static bool bridge_start(Bridge* b) {
    ring_reset(&b->ring);
    OSStatus err = AudioDeviceCreateIOProcID(b->realDev, real_out_io_proc, b, &b->realProc);
    if (err != noErr || !b->realProc) {
        VMERR("bridge_start: real CreateIOProcID failed: %{public}d", (int)err);
        return false;
    }
    err = AudioDeviceCreateIOProcID(b->virtDev, virt_in_io_proc, b, &b->virtProc);
    if (err != noErr || !b->virtProc) {
        VMERR("bridge_start: virt CreateIOProcID failed: %{public}d", (int)err);
        AudioDeviceDestroyIOProcID(b->realDev, b->realProc); b->realProc = NULL;
        return false;
    }
    err = AudioDeviceStart(b->realDev, b->realProc);
    if (err != noErr) {
        VMERR("bridge_start: real Start failed: %{public}d", (int)err);
        AudioDeviceDestroyIOProcID(b->realDev, b->realProc); b->realProc = NULL;
        AudioDeviceDestroyIOProcID(b->virtDev, b->virtProc); b->virtProc = NULL;
        return false;
    }
    err = AudioDeviceStart(b->virtDev, b->virtProc);
    if (err != noErr) {
        VMERR("bridge_start: virt Start failed: %{public}d", (int)err);
        AudioDeviceStop(b->realDev, b->realProc);
        AudioDeviceDestroyIOProcID(b->realDev, b->realProc); b->realProc = NULL;
        AudioDeviceDestroyIOProcID(b->virtDev, b->virtProc); b->virtProc = NULL;
        return false;
    }
    b->active = true;
    VMLOG("bridge up: virt=%{public}u → real=%{public}u (%{public}s)",
          (unsigned)b->virtDev, (unsigned)b->realDev, b->realUID);
    return true;
}

static void bridge_stop(Bridge* b) {
    if (!b->active) return;
    if (b->virtProc) {
        AudioDeviceStop(b->virtDev, b->virtProc);
        AudioDeviceDestroyIOProcID(b->virtDev, b->virtProc);
        b->virtProc = NULL;
    }
    if (b->realProc) {
        AudioDeviceStop(b->realDev, b->realProc);
        AudioDeviceDestroyIOProcID(b->realDev, b->realProc);
        b->realProc = NULL;
    }
    b->active = false;
    VMLOG("bridge down: virt=%{public}u real=%{public}u", (unsigned)b->virtDev, (unsigned)b->realDev);
}

// ---------- Reconcile ----------

// Find a live VolMirror virtual device whose target either matches `realUID`
// (already assigned) or is empty (free for us to assign).
typedef struct { AudioObjectID dev; char uid[256]; char curTarget[224]; } VirtSlot;

static int collect_virt_slots(VirtSlot* out, int cap) {
    UInt32 nDev = 0;
    AudioObjectID* devs = enumerate_devices(&nDev);
    if (!devs) return 0;
    int n = 0;
    for (UInt32 i = 0; i < nDev && n < cap; i++) {
        char uid[256] = {0};
        get_cfstring(devs[i], kAudioDevicePropertyDeviceUID,
                     kAudioObjectPropertyScopeGlobal, uid, sizeof(uid));
        if (strncmp(uid, "VolMirror/", 10) != 0) continue;
        out[n].dev = devs[i];
        snprintf(out[n].uid, sizeof(out[n].uid), "%s", uid);
        get_cfstring(devs[i], kVMProperty_FollowTarget,
                     kAudioObjectPropertyScopeGlobal,
                     out[n].curTarget, sizeof(out[n].curTarget));
        n++;
    }
    free(devs);
    return n;
}

static Bridge* find_bridge_by_real_uid(const char* uid) {
    for (int i = 0; i < kMaxBridges; i++)
        if (gBridges[i].active && strcmp(gBridges[i].realUID, uid) == 0) return &gBridges[i];
    return NULL;
}

static Bridge* find_free_bridge_slot(void) {
    for (int i = 0; i < kMaxBridges; i++) if (!gBridges[i].active) return &gBridges[i];
    return NULL;
}

static void reconcile(void) {
    UInt32 nDev = 0;
    AudioObjectID* devs = enumerate_devices(&nDev);
    if (!devs) return;

    // 1. Snapshot current locked-volume real outputs.
    typedef struct { AudioObjectID id; char uid[256]; char name[256]; } Real;
    Real reals[kMaxBridges * 2];
    int nReal = 0;
    int realCap = (int)(sizeof(reals)/sizeof(reals[0]));
    for (UInt32 i = 0; i < nDev && nReal < realCap; i++) {
        if (!real_is_output_with_locked_volume(devs[i])) continue;
        Real* r = &reals[nReal];
        r->id = devs[i];
        get_cfstring(devs[i], kAudioDevicePropertyDeviceUID,
                     kAudioObjectPropertyScopeGlobal, r->uid, sizeof(r->uid));
        get_cfstring(devs[i], kAudioObjectPropertyName,
                     kAudioObjectPropertyScopeGlobal, r->name, sizeof(r->name));
        if (r->name[0] == 0) snprintf(r->name, sizeof(r->name), "Output %u", (unsigned)devs[i]);
        if (r->uid[0]) nReal++;
    }
    free(devs);

    // 2. Tear down bridges whose real device disappeared.
    for (int i = 0; i < kMaxBridges; i++) {
        if (!gBridges[i].active) continue;
        bool present = false;
        for (int j = 0; j < nReal; j++) {
            if (strcmp(gBridges[i].realUID, reals[j].uid) == 0) {
                gBridges[i].realDev = reals[j].id;       // refresh transient ID
                present = true; break;
            }
        }
        if (!present) {
            // Clear the slot's assignment so the OS hides the virtual device,
            // then tear down our IOProcs.
            set_cfstring(gBridges[i].virtDev, kVMProperty_FollowTarget, "");
            set_cfstring(gBridges[i].virtDev, kVMProperty_FollowName, "");
            bridge_stop(&gBridges[i]);
            gBridges[i].realUID[0] = 0;
            gBridges[i].virtUID[0] = 0;
        }
    }

    // 3. Find the driver's virtual slots (state on the driver side).
    VirtSlot virts[kMaxBridges * 2];
    int nVirt = collect_virt_slots(virts, (int)(sizeof(virts)/sizeof(virts[0])));

    // 4. For each present locked real output, ensure a bridge exists.
    for (int j = 0; j < nReal; j++) {
        if (find_bridge_by_real_uid(reals[j].uid)) continue;     // already up

        // Pick a virtual slot: prefer one whose target already matches us
        // (driver remembers across coreaudiod restarts), else first unassigned.
        VirtSlot* picked = NULL;
        for (int k = 0; k < nVirt; k++)
            if (strcmp(virts[k].curTarget, reals[j].uid) == 0) { picked = &virts[k]; break; }
        if (!picked)
            for (int k = 0; k < nVirt; k++)
                if (virts[k].curTarget[0] == 0) { picked = &virts[k]; break; }
        if (!picked) {
            VMLOG("no free virtual slot for %{public}s", reals[j].name);
            continue;
        }

        Bridge* b = find_free_bridge_slot();
        if (!b) {
            VMLOG("no free bridge slot for %{public}s", reals[j].name);
            continue;
        }

        // Assign on the driver side first so the virtual device shows in UI.
        char displayName[300];
        snprintf(displayName, sizeof(displayName), "%s (Vol)", reals[j].name);
        set_cfstring(picked->dev, kVMProperty_FollowName, displayName);
        set_cfstring(picked->dev, kVMProperty_FollowTarget, reals[j].uid);

        // Wire up the bridge.
        b->virtDev = picked->dev;
        b->realDev = reals[j].id;
        snprintf(b->realUID, sizeof(b->realUID), "%s", reals[j].uid);
        snprintf(b->virtUID, sizeof(b->virtUID), "%s", picked->uid);
        ring_init(&b->ring, kAgentRingFrames);
        if (!bridge_start(b)) {
            // Roll back the driver assignment.
            set_cfstring(picked->dev, kVMProperty_FollowTarget, "");
            set_cfstring(picked->dev, kVMProperty_FollowName, "");
            free(b->ring.data); b->ring.data = NULL;
            memset(b, 0, sizeof(*b));
        }
    }
}

// ---------- Hotplug debounce ----------
static void schedule_reconcile(void) {
    if (!gDebounce) return;
    dispatch_source_set_timer(gDebounce,
        dispatch_time(DISPATCH_TIME_NOW, 100 * NSEC_PER_MSEC),
        DISPATCH_TIME_FOREVER, 10 * NSEC_PER_MSEC);
}

// ---------- Shutdown ----------
static void shutdown_all(void) {
    if (atomic_exchange(&gShutdown, 1)) return;
    VMLOG("shutdown");
    for (int i = 0; i < kMaxBridges; i++) {
        if (!gBridges[i].active) continue;
        set_cfstring(gBridges[i].virtDev, kVMProperty_FollowTarget, "");
        set_cfstring(gBridges[i].virtDev, kVMProperty_FollowName, "");
        bridge_stop(&gBridges[i]);
        if (gBridges[i].ring.data) { free(gBridges[i].ring.data); gBridges[i].ring.data = NULL; }
    }
}

static void on_signal(int sig) {
    (void)sig;
    shutdown_all();
    _exit(0);
}

int main(void) {
    VMLOG("VolMirrorAgent starting (pid=%{public}d)", getpid());

    signal(SIGTERM, on_signal);
    signal(SIGINT,  on_signal);
    signal(SIGHUP,  on_signal);

    gReconcileQ = dispatch_queue_create("com.joaopedro.VolMirror.agent.reconcile",
                                         DISPATCH_QUEUE_SERIAL);
    gDebounce = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, gReconcileQ);
    dispatch_source_set_event_handler(gDebounce, ^{ reconcile(); });
    dispatch_resume(gDebounce);

    static const AudioObjectPropertyAddress kDevicesAddr = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain,
    };
    OSStatus err = AudioObjectAddPropertyListenerBlock(
        kAudioObjectSystemObject, &kDevicesAddr, gReconcileQ,
        ^(UInt32 n, const AudioObjectPropertyAddress* addrs) {
            (void)n; (void)addrs;
            schedule_reconcile();
        });
    if (err != noErr) VMERR("AddPropertyListenerBlock failed: %{public}d", (int)err);

    // Initial scan, after a small delay to let coreaudiod finish loading
    // the driver if the agent and driver were installed in the same step.
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 500 * NSEC_PER_MSEC),
                   gReconcileQ, ^{ reconcile(); });

    dispatch_main();
    return 0;
}
