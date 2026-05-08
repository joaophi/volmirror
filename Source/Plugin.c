// VolMirror — CoreAudio Server Plugin (thin)
//
// N pre-allocated duplex virtual loopback slots. Each slot:
//   - apps write audio to the output stream → driver applies volume/mute → ring
//   - companion agent reads the ring via the input stream → forwards to a real device
// Slots are hidden by default. The agent assigns a slot to a real device by
// writing custom properties:
//   'flwt' (CFString) — real device UID; "" unassigns
//   'fldn' (CFString) — display name shown in the UI as "<name> (Vol)"
// The driver makes zero client-HAL calls. All routing logic lives in the
// agent, so iterating on it doesn't require killing coreaudiod.

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreFoundation/CFPlugInCOM.h>
#include <CoreFoundation/CoreFoundation.h>
#include <mach/mach_time.h>
#include <os/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define VMLOG(fmt, ...) os_log(OS_LOG_DEFAULT, "[VolMirror] " fmt, ##__VA_ARGS__)

#define kPlugInObjectID         kAudioObjectPlugInObject
#define kMaxMirrors             8
#define kFirstMirrorObjectID    ((AudioObjectID)100)
#define kDefaultSampleRate      48000.0
#define kZeroTimeStampPeriod    16384    // ≥ 10923 per AudioServerPlugIn.h
#define kRingCapacity           4096     // frames, power of 2

// Custom property selectors. CFString-valued.
#define kVMProperty_FollowTarget    'flwt'
#define kVMProperty_FollowName      'fldn'

static inline AudioObjectID dev_id(int i)    { return kFirstMirrorObjectID + i*10 + 0; }
static inline AudioObjectID outstrm_id(int i){ return kFirstMirrorObjectID + i*10 + 1; }
static inline AudioObjectID instrm_id(int i) { return kFirstMirrorObjectID + i*10 + 2; }
static inline AudioObjectID volc_id(int i)   { return kFirstMirrorObjectID + i*10 + 3; }
static inline AudioObjectID mute_id(int i)   { return kFirstMirrorObjectID + i*10 + 4; }

static int object_to_mirror(AudioObjectID o) {
    if (o < kFirstMirrorObjectID) return -1;
    int i = (int)((o - kFirstMirrorObjectID) / 10);
    if (i < 0 || i >= kMaxMirrors) return -1;
    return i;
}

// SPSC ring (stereo float32, interleaved). Power-of-two capacity in frames.
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

// Producer: write `frames` of stereo Float32, linearly ramping gain to avoid clicks.
static UInt32 ring_write_scaled(Ring* r, const float* src, UInt32 frames,
                                float gainStart, float gainEnd) {
    if (!r->data) return 0;
    UInt32 mask = r->capacity - 1;
    UInt32 head = atomic_load_explicit(&r->head, memory_order_acquire);
    UInt32 tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    UInt32 used = (tail - head) & mask;
    UInt32 freeSpace = r->capacity - 1 - used;
    UInt32 toWrite = frames < freeSpace ? frames : freeSpace;
    if (toWrite == 0) return 0;
    float step = (toWrite > 1) ? (gainEnd - gainStart) / (float)(toWrite - 1) : 0.0f;
    for (UInt32 i = 0; i < toWrite; i++) {
        UInt32 idx = (tail + i) & mask;
        float g = gainStart + step * (float)i;
        r->data[idx*2 + 0] = src[i*2 + 0] * g;
        r->data[idx*2 + 1] = src[i*2 + 1] * g;
    }
    atomic_store_explicit(&r->tail, (tail + toWrite) & mask, memory_order_release);
    return toWrite;
}

// Consumer: drain up to `frames` into stereo dst (interleaved). Underrun → silence.
static UInt32 ring_read_stereo(Ring* r, float* dst, UInt32 frames) {
    if (!r->data) { memset(dst, 0, frames * 2 * sizeof(float)); return 0; }
    UInt32 mask = r->capacity - 1;
    UInt32 head = atomic_load_explicit(&r->head, memory_order_relaxed);
    UInt32 tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    UInt32 used = (tail - head) & mask;
    UInt32 toRead = frames < used ? frames : used;
    for (UInt32 i = 0; i < toRead; i++) {
        UInt32 idx = (head + i) & mask;
        dst[i*2 + 0] = r->data[idx*2 + 0];
        dst[i*2 + 1] = r->data[idx*2 + 1];
    }
    if (toRead < frames)
        memset(dst + toRead * 2, 0, (frames - toRead) * 2 * sizeof(float));
    atomic_store_explicit(&r->head, (head + toRead) & mask, memory_order_release);
    return toRead;
}

// Per-slot state.
typedef struct {
    char                realUID[224];       // "" when unassigned
    char                displayName[300];
    char                uid[260];           // synthesized; stable when unassigned
    _Atomic bool        assigned;
    Float64             sampleRate;
    _Atomic float       volume;             // 0..1
    _Atomic bool        muted;
    float               currentGain;        // producer-thread only
    _Atomic UInt32      ioRunning;          // client refcount
    UInt64              ioStartHost;
    Ring                ring;
    pthread_mutex_t     mu;                 // assignment + IO lifecycle
} Mirror;

static Mirror                       gMirrors[kMaxMirrors];
static AudioServerPlugInHostRef     gHost = NULL;
static atomic_uint                  gRefCount = 0;
static mach_timebase_info_data_t    gTb;

static AudioStreamBasicDescription asbd_for(Float64 rate) {
    AudioStreamBasicDescription a = {0};
    a.mSampleRate       = rate;
    a.mFormatID         = kAudioFormatLinearPCM;
    a.mFormatFlags      = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian
                        | kAudioFormatFlagIsPacked;
    a.mFramesPerPacket  = 1;
    a.mChannelsPerFrame = 2;
    a.mBitsPerChannel   = 32;
    a.mBytesPerFrame    = 8;
    a.mBytesPerPacket   = 8;
    return a;
}

// Volume taper.
static const Float32 kMinDB = -96.0f;
static inline float scalar_to_gain(float s) {
    if (s <= 0.0f) return 0.0f;
    if (s >= 1.0f) return 1.0f;
    return s * s;
}
static Float32 scalar_to_db(Float32 s) {
    if (s <= 0.0f) return kMinDB;
    if (s >= 1.0f) return 0.0f;
    Float32 db = 40.0f * log10f(s);
    return db < kMinDB ? kMinDB : db;
}
static Float32 db_to_scalar(Float32 d) {
    if (!(d > kMinDB)) return 0.0f;
    if (d >= 0.0f)    return 1.0f;
    return powf(10.0f, d / 40.0f);
}

// Re-derive synthesized fields from realUID. Caller holds m->mu.
static void slot_resync(Mirror* m, int i) {
    if (m->realUID[0]) {
        snprintf(m->uid, sizeof(m->uid), "VolMirror/%s", m->realUID);
        atomic_store_explicit(&m->assigned, true, memory_order_release);
    } else {
        snprintf(m->uid, sizeof(m->uid), "VolMirror/slot/%d", i);
        atomic_store_explicit(&m->assigned, false, memory_order_release);
    }
}

// ===========================================================================
// V-table forward decls
// ===========================================================================
static HRESULT  VM_QueryInterface(void*, REFIID, LPVOID*);
static ULONG    VM_AddRef(void*);
static ULONG    VM_Release(void*);
static OSStatus VM_Initialize(AudioServerPlugInDriverRef, AudioServerPlugInHostRef);
static OSStatus VM_CreateDevice(AudioServerPlugInDriverRef, CFDictionaryRef,
                                const AudioServerPlugInClientInfo*, AudioObjectID*);
static OSStatus VM_DestroyDevice(AudioServerPlugInDriverRef, AudioObjectID);
static OSStatus VM_AddDeviceClient(AudioServerPlugInDriverRef, AudioObjectID,
                                   const AudioServerPlugInClientInfo*);
static OSStatus VM_RemoveDeviceClient(AudioServerPlugInDriverRef, AudioObjectID,
                                      const AudioServerPlugInClientInfo*);
static OSStatus VM_PerformDeviceConfigChange(AudioServerPlugInDriverRef, AudioObjectID,
                                             UInt64, void*);
static OSStatus VM_AbortDeviceConfigChange(AudioServerPlugInDriverRef, AudioObjectID,
                                           UInt64, void*);
static Boolean  VM_HasProperty(AudioServerPlugInDriverRef, AudioObjectID, pid_t,
                               const AudioObjectPropertyAddress*);
static OSStatus VM_IsPropertySettable(AudioServerPlugInDriverRef, AudioObjectID, pid_t,
                                      const AudioObjectPropertyAddress*, Boolean*);
static OSStatus VM_GetPropertyDataSize(AudioServerPlugInDriverRef, AudioObjectID, pid_t,
                                       const AudioObjectPropertyAddress*, UInt32,
                                       const void*, UInt32*);
static OSStatus VM_GetPropertyData(AudioServerPlugInDriverRef, AudioObjectID, pid_t,
                                   const AudioObjectPropertyAddress*, UInt32,
                                   const void*, UInt32, UInt32*, void*);
static OSStatus VM_SetPropertyData(AudioServerPlugInDriverRef, AudioObjectID, pid_t,
                                   const AudioObjectPropertyAddress*, UInt32,
                                   const void*, UInt32, const void*);
static OSStatus VM_StartIO(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
static OSStatus VM_StopIO(AudioServerPlugInDriverRef, AudioObjectID, UInt32);
static OSStatus VM_GetZeroTimeStamp(AudioServerPlugInDriverRef, AudioObjectID, UInt32,
                                    Float64*, UInt64*, UInt64*);
static OSStatus VM_WillDoIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32,
                                     UInt32, Boolean*, Boolean*);
static OSStatus VM_BeginIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32,
                                    UInt32, UInt32, const AudioServerPlugInIOCycleInfo*);
static OSStatus VM_DoIOOperation(AudioServerPlugInDriverRef, AudioObjectID, AudioObjectID,
                                 UInt32, UInt32, UInt32,
                                 const AudioServerPlugInIOCycleInfo*, void*, void*);
static OSStatus VM_EndIOOperation(AudioServerPlugInDriverRef, AudioObjectID, UInt32,
                                  UInt32, UInt32, const AudioServerPlugInIOCycleInfo*);

static AudioServerPlugInDriverInterface gInterface = {
    NULL,
    VM_QueryInterface, VM_AddRef, VM_Release,
    VM_Initialize, VM_CreateDevice, VM_DestroyDevice,
    VM_AddDeviceClient, VM_RemoveDeviceClient,
    VM_PerformDeviceConfigChange, VM_AbortDeviceConfigChange,
    VM_HasProperty, VM_IsPropertySettable,
    VM_GetPropertyDataSize, VM_GetPropertyData, VM_SetPropertyData,
    VM_StartIO, VM_StopIO, VM_GetZeroTimeStamp,
    VM_WillDoIOOperation, VM_BeginIOOperation, VM_DoIOOperation, VM_EndIOOperation,
};
static AudioServerPlugInDriverInterface* gInterfacePtr = &gInterface;

// ===========================================================================
// Property helpers
// ===========================================================================
#define WRITE(type, val)                                              \
    do {                                                              \
        if (outDataSize) *outDataSize = sizeof(type);                 \
        if (dataSize >= sizeof(type)) *(type*)outData = (val);        \
        else return kAudioHardwareBadPropertySizeError;               \
        return noErr;                                                 \
    } while (0)

#define WRITE_CSTR(s)                                                 \
    do {                                                              \
        if (outDataSize) *outDataSize = sizeof(CFStringRef);          \
        if (dataSize >= sizeof(CFStringRef))                          \
            *(CFStringRef*)outData =                                  \
                CFStringCreateWithCString(NULL, (s), kCFStringEncodingUTF8);\
        else return kAudioHardwareBadPropertySizeError;               \
        return noErr;                                                 \
    } while (0)

// ===========================================================================
// Plugin object
// ===========================================================================
static OSStatus get_plugin_prop_size(const AudioObjectPropertyAddress* a, UInt32* outSize) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyManufacturer:
        case kAudioPlugInPropertyResourceBundle:
            *outSize = sizeof(CFStringRef); return noErr;
        case kAudioPlugInPropertyBoxList:
            *outSize = 0; return noErr;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList:
            // Always advertise all slots; visibility is per-device via IsHidden.
            *outSize = (UInt32)(kMaxMirrors * sizeof(AudioObjectID));
            return noErr;
        case kAudioPlugInPropertyTranslateUIDToBox:
        case kAudioPlugInPropertyTranslateUIDToDevice:
            *outSize = sizeof(AudioObjectID); return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_plugin_prop(const AudioObjectPropertyAddress* a, UInt32 dataSize,
                                const void* qual, UInt32* outDataSize, void* outData) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
            WRITE(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass:
            WRITE(AudioClassID, kAudioPlugInClassID);
        case kAudioObjectPropertyOwner:
            WRITE(AudioObjectID, kAudioObjectUnknown);
        case kAudioObjectPropertyManufacturer:
            WRITE_CSTR("VolMirror");
        case kAudioPlugInPropertyResourceBundle:
            if (outDataSize) *outDataSize = sizeof(CFStringRef);
            if (dataSize >= sizeof(CFStringRef))
                *(CFStringRef*)outData = CFStringCreateCopy(NULL, CFSTR(""));
            else return kAudioHardwareBadPropertySizeError;
            return noErr;
        case kAudioPlugInPropertyBoxList:
            if (outDataSize) *outDataSize = 0;
            return noErr;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList: {
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            UInt32 n = 0;
            AudioObjectID* dst = (AudioObjectID*)outData;
            for (int i = 0; i < kMaxMirrors && n < cap; i++)
                dst[n++] = dev_id(i);
            if (outDataSize) *outDataSize = (UInt32)(n * sizeof(AudioObjectID));
            return noErr;
        }
        case kAudioPlugInPropertyTranslateUIDToBox:
            WRITE(AudioObjectID, kAudioObjectUnknown);
        case kAudioPlugInPropertyTranslateUIDToDevice: {
            if (qual == NULL) return kAudioHardwareBadPropertySizeError;
            CFStringRef in = *(CFStringRef*)qual;
            AudioObjectID found = kAudioObjectUnknown;
            for (int i = 0; i < kMaxMirrors; i++) {
                pthread_mutex_lock(&gMirrors[i].mu);
                CFStringRef mine = CFStringCreateWithCString(NULL, gMirrors[i].uid,
                                                              kCFStringEncodingUTF8);
                if (CFStringCompare(in, mine, 0) == kCFCompareEqualTo) found = dev_id(i);
                CFRelease(mine);
                pthread_mutex_unlock(&gMirrors[i].mu);
                if (found != kAudioObjectUnknown) break;
            }
            WRITE(AudioObjectID, found);
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Device object
// ===========================================================================
static UInt32 streams_for_scope(int idx, AudioObjectPropertyScope scope,
                                AudioObjectID* out, UInt32 cap) {
    UInt32 n = 0;
    if (scope == kAudioObjectPropertyScopeOutput || scope == kAudioObjectPropertyScopeGlobal) {
        if (cap > n) out[n] = outstrm_id(idx); n++;
    }
    if (scope == kAudioObjectPropertyScopeInput || scope == kAudioObjectPropertyScopeGlobal) {
        if (cap > n) out[n] = instrm_id(idx); n++;
    }
    return n;
}

static OSStatus get_device_prop_size(int idx, const AudioObjectPropertyAddress* a,
                                     UInt32* outSize) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyName:
        case kAudioObjectPropertyManufacturer:
        case kAudioDevicePropertyDeviceUID:
        case kAudioDevicePropertyModelUID:
            *outSize = sizeof(CFStringRef); return noErr;
        case kAudioDevicePropertyTransportType:
        case kAudioDevicePropertyClockDomain:
        case kAudioDevicePropertyClockAlgorithm:
        case kAudioDevicePropertyClockIsStable:
        case kAudioDevicePropertyDeviceIsAlive:
        case kAudioDevicePropertyDeviceIsRunning:
        case kAudioDevicePropertyIsHidden:
        case kAudioDevicePropertyZeroTimeStampPeriod:
        case kAudioDevicePropertyLatency:
        case kAudioDevicePropertySafetyOffset:
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
            *outSize = sizeof(UInt32); return noErr;
        case kAudioDevicePropertyNominalSampleRate:
            *outSize = sizeof(Float64); return noErr;
        case kAudioDevicePropertyAvailableNominalSampleRates:
            *outSize = sizeof(AudioValueRange); return noErr;
        case kAudioDevicePropertyPreferredChannelsForStereo:
            *outSize = 2 * sizeof(UInt32); return noErr;
        case kAudioObjectPropertyOwnedObjects: {
            if (a->mScope == kAudioObjectPropertyScopeInput) {
                *outSize = sizeof(AudioObjectID);          // input stream
            } else if (a->mScope == kAudioObjectPropertyScopeOutput) {
                *outSize = 3 * sizeof(AudioObjectID);      // output stream + vol + mute
            } else {
                *outSize = 4 * sizeof(AudioObjectID);      // both streams + vol + mute
            }
            return noErr;
        }
        case kAudioObjectPropertyControlList:
            *outSize = 2 * sizeof(AudioObjectID);
            return noErr;
        case kAudioDevicePropertyStreams: {
            UInt32 n = streams_for_scope(idx, a->mScope, NULL, 0);
            *outSize = n * sizeof(AudioObjectID); return noErr;
        }
        case kAudioDevicePropertyRelatedDevices:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyCustomPropertyInfoList:
            *outSize = 2 * sizeof(AudioServerPlugInCustomPropertyInfo);
            return noErr;
        case kVMProperty_FollowTarget:
        case kVMProperty_FollowName:
            *outSize = sizeof(CFStringRef); return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_device_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, UInt32* outDataSize, void* outData) {
    Mirror* m = &gMirrors[idx];
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
            WRITE(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass:
            WRITE(AudioClassID, kAudioDeviceClassID);
        case kAudioObjectPropertyOwner:
            WRITE(AudioObjectID, kPlugInObjectID);
        case kAudioObjectPropertyName: {
            pthread_mutex_lock(&m->mu);
            char buf[300]; snprintf(buf, sizeof(buf), "%s", m->displayName);
            pthread_mutex_unlock(&m->mu);
            WRITE_CSTR(buf);
        }
        case kAudioObjectPropertyManufacturer:
            WRITE_CSTR("VolMirror");
        case kAudioDevicePropertyDeviceUID: {
            pthread_mutex_lock(&m->mu);
            char buf[260]; snprintf(buf, sizeof(buf), "%s", m->uid);
            pthread_mutex_unlock(&m->mu);
            WRITE_CSTR(buf);
        }
        case kAudioDevicePropertyModelUID:
            WRITE_CSTR("com.joaopedro.VolMirror.model");
        case kAudioDevicePropertyTransportType:
            WRITE(UInt32, kAudioDeviceTransportTypeVirtual);
        case kAudioDevicePropertyClockDomain:
            WRITE(UInt32, 0);
        case kAudioDevicePropertyClockAlgorithm:
            WRITE(UInt32, kAudioDeviceClockAlgorithmSimpleIIR);
        case kAudioDevicePropertyClockIsStable:
            WRITE(UInt32, 1);
        case kAudioDevicePropertyDeviceIsAlive:
            WRITE(UInt32, 1);
        case kAudioDevicePropertyDeviceIsRunning:
            WRITE(UInt32, atomic_load_explicit(&m->ioRunning, memory_order_relaxed) > 0 ? 1 : 0);
        case kAudioDevicePropertyIsHidden:
            WRITE(UInt32, atomic_load_explicit(&m->assigned, memory_order_acquire) ? 0 : 1);
        case kAudioDevicePropertyZeroTimeStampPeriod:
            WRITE(UInt32, kZeroTimeStampPeriod);
        case kAudioDevicePropertyLatency:
            WRITE(UInt32, 1024);
        case kAudioDevicePropertySafetyOffset:
            WRITE(UInt32, 0);
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
            WRITE(UInt32, 1);
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
            WRITE(UInt32, 1);
        case kAudioDevicePropertyNominalSampleRate:
            WRITE(Float64, m->sampleRate);
        case kAudioDevicePropertyAvailableNominalSampleRates: {
            if (outDataSize) *outDataSize = sizeof(AudioValueRange);
            if (dataSize >= sizeof(AudioValueRange)) {
                AudioValueRange* r = (AudioValueRange*)outData;
                r->mMinimum = m->sampleRate; r->mMaximum = m->sampleRate;
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
        case kAudioDevicePropertyPreferredChannelsForStereo:
            if (outDataSize) *outDataSize = 2 * sizeof(UInt32);
            if (dataSize >= 2 * sizeof(UInt32)) {
                ((UInt32*)outData)[0] = 1; ((UInt32*)outData)[1] = 2;
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        case kAudioObjectPropertyOwnedObjects: {
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            AudioObjectID* dst = (AudioObjectID*)outData;
            UInt32 n = 0;
            if (a->mScope == kAudioObjectPropertyScopeInput) {
                if (cap > n) dst[n] = instrm_id(idx); n++;
            } else if (a->mScope == kAudioObjectPropertyScopeOutput) {
                if (cap > n) { dst[n] = outstrm_id(idx); } n++;
                if (cap > n) { dst[n] = volc_id(idx);    } n++;
                if (cap > n) { dst[n] = mute_id(idx);    } n++;
            } else {
                if (cap > n) { dst[n] = outstrm_id(idx); } n++;
                if (cap > n) { dst[n] = instrm_id(idx);  } n++;
                if (cap > n) { dst[n] = volc_id(idx);    } n++;
                if (cap > n) { dst[n] = mute_id(idx);    } n++;
            }
            if (outDataSize) *outDataSize = n * sizeof(AudioObjectID);
            return noErr;
        }
        case kAudioObjectPropertyControlList: {
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            AudioObjectID* dst = (AudioObjectID*)outData;
            UInt32 n = 0;
            if (cap > n) { dst[n] = volc_id(idx); } n++;
            if (cap > n) { dst[n] = mute_id(idx); } n++;
            if (outDataSize) *outDataSize = n * sizeof(AudioObjectID);
            return noErr;
        }
        case kAudioDevicePropertyStreams: {
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            UInt32 n = streams_for_scope(idx, a->mScope, (AudioObjectID*)outData, cap);
            if (outDataSize) *outDataSize = n * sizeof(AudioObjectID);
            return noErr;
        }
        case kAudioDevicePropertyRelatedDevices:
            if (outDataSize) *outDataSize = sizeof(AudioObjectID);
            if (dataSize >= sizeof(AudioObjectID)) {
                *(AudioObjectID*)outData = dev_id(idx); return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        case kAudioObjectPropertyCustomPropertyInfoList: {
            if (outDataSize) *outDataSize = 2 * sizeof(AudioServerPlugInCustomPropertyInfo);
            if (dataSize >= 2 * sizeof(AudioServerPlugInCustomPropertyInfo)) {
                AudioServerPlugInCustomPropertyInfo* info =
                    (AudioServerPlugInCustomPropertyInfo*)outData;
                info[0].mSelector          = kVMProperty_FollowTarget;
                info[0].mPropertyDataType  = kAudioServerPlugInCustomPropertyDataTypeCFString;
                info[0].mQualifierDataType = kAudioServerPlugInCustomPropertyDataTypeNone;
                info[1].mSelector          = kVMProperty_FollowName;
                info[1].mPropertyDataType  = kAudioServerPlugInCustomPropertyDataTypeCFString;
                info[1].mQualifierDataType = kAudioServerPlugInCustomPropertyDataTypeNone;
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
        case kVMProperty_FollowTarget: {
            pthread_mutex_lock(&m->mu);
            char buf[224]; snprintf(buf, sizeof(buf), "%s", m->realUID);
            pthread_mutex_unlock(&m->mu);
            WRITE_CSTR(buf);
        }
        case kVMProperty_FollowName: {
            pthread_mutex_lock(&m->mu);
            char buf[300]; snprintf(buf, sizeof(buf), "%s", m->displayName);
            pthread_mutex_unlock(&m->mu);
            WRITE_CSTR(buf);
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// Notify the host that the slot's identity (or visibility) changed. Called
// after assignment changes; lets the OS UI re-query name/UID/IsHidden.
static void notify_slot_changed(int idx) {
    if (!gHost) return;
    static const AudioObjectPropertyAddress addrs[] = {
        { kAudioObjectPropertyName,           kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
        { kAudioDevicePropertyDeviceUID,      kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
        { kAudioDevicePropertyIsHidden,       kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
        { kVMProperty_FollowTarget,           kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
        { kVMProperty_FollowName,             kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
    };
    gHost->PropertiesChanged(gHost, dev_id(idx),
                             sizeof(addrs)/sizeof(addrs[0]), addrs);
}

static OSStatus set_device_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, const void* data) {
    Mirror* m = &gMirrors[idx];
    switch (a->mSelector) {
        case kVMProperty_FollowTarget: {
            if (dataSize < sizeof(CFStringRef) || data == NULL)
                return kAudioHardwareBadPropertySizeError;
            CFStringRef s = *(CFStringRef*)data;
            char buf[224] = {0};
            if (s) CFStringGetCString(s, buf, sizeof(buf), kCFStringEncodingUTF8);
            pthread_mutex_lock(&m->mu);
            snprintf(m->realUID, sizeof(m->realUID), "%s", buf);
            slot_resync(m, idx);
            pthread_mutex_unlock(&m->mu);
            VMLOG("slot %{public}d follow → %{public}s", idx, buf[0] ? buf : "(unassigned)");
            notify_slot_changed(idx);
            return noErr;
        }
        case kVMProperty_FollowName: {
            if (dataSize < sizeof(CFStringRef) || data == NULL)
                return kAudioHardwareBadPropertySizeError;
            CFStringRef s = *(CFStringRef*)data;
            char buf[300] = {0};
            if (s) CFStringGetCString(s, buf, sizeof(buf), kCFStringEncodingUTF8);
            pthread_mutex_lock(&m->mu);
            if (buf[0]) snprintf(m->displayName, sizeof(m->displayName), "%s", buf);
            else        snprintf(m->displayName, sizeof(m->displayName), "VolMirror Slot %d", idx);
            pthread_mutex_unlock(&m->mu);
            notify_slot_changed(idx);
            return noErr;
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Stream object
// ===========================================================================
static bool is_input_stream(int idx, AudioObjectID o) { return o == instrm_id(idx); }

static OSStatus get_stream_prop_size(const AudioObjectPropertyAddress* a, UInt32* outSize) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyOwnedObjects:
            *outSize = 0; return noErr;
        case kAudioStreamPropertyIsActive:
        case kAudioStreamPropertyDirection:
        case kAudioStreamPropertyTerminalType:
        case kAudioStreamPropertyStartingChannel:
        case kAudioStreamPropertyLatency:
            *outSize = sizeof(UInt32); return noErr;
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat:
            *outSize = sizeof(AudioStreamBasicDescription); return noErr;
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyAvailablePhysicalFormats:
            *outSize = sizeof(AudioStreamRangedDescription); return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_stream_prop(int idx, AudioObjectID o,
                                const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, UInt32* outDataSize, void* outData) {
    bool isIn = is_input_stream(idx, o);
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
            WRITE(AudioClassID, kAudioObjectClassID);
        case kAudioObjectPropertyClass:
            WRITE(AudioClassID, kAudioStreamClassID);
        case kAudioObjectPropertyOwner:
            WRITE(AudioObjectID, dev_id(idx));
        case kAudioObjectPropertyOwnedObjects:
            if (outDataSize) *outDataSize = 0; return noErr;
        case kAudioStreamPropertyIsActive:
            WRITE(UInt32, 1);
        case kAudioStreamPropertyDirection:
            WRITE(UInt32, isIn ? 1u : 0u);
        case kAudioStreamPropertyTerminalType:
            WRITE(UInt32, kAudioStreamTerminalTypeLine);
        case kAudioStreamPropertyStartingChannel:
            WRITE(UInt32, 1);
        case kAudioStreamPropertyLatency:
            WRITE(UInt32, 0);
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyPhysicalFormat: {
            if (outDataSize) *outDataSize = sizeof(AudioStreamBasicDescription);
            if (dataSize >= sizeof(AudioStreamBasicDescription)) {
                AudioStreamBasicDescription a = asbd_for(gMirrors[idx].sampleRate);
                memcpy(outData, &a, sizeof(a));
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyAvailablePhysicalFormats: {
            if (outDataSize) *outDataSize = sizeof(AudioStreamRangedDescription);
            if (dataSize >= sizeof(AudioStreamRangedDescription)) {
                Float64 rate = gMirrors[idx].sampleRate;
                AudioStreamRangedDescription r = {0};
                r.mFormat = asbd_for(rate);
                r.mSampleRateRange.mMinimum = rate;
                r.mSampleRateRange.mMaximum = rate;
                memcpy(outData, &r, sizeof(r));
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Volume control
// ===========================================================================
static OSStatus get_volume_prop_size(const AudioObjectPropertyAddress* a, UInt32* outSize) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyOwnedObjects:
            *outSize = 0; return noErr;
        case kAudioControlPropertyScope:
            *outSize = sizeof(AudioObjectPropertyScope); return noErr;
        case kAudioControlPropertyElement:
            *outSize = sizeof(AudioObjectPropertyElement); return noErr;
        case kAudioLevelControlPropertyScalarValue:
        case kAudioLevelControlPropertyDecibelValue:
        case kAudioLevelControlPropertyConvertScalarToDecibels:
        case kAudioLevelControlPropertyConvertDecibelsToScalar:
            *outSize = sizeof(Float32); return noErr;
        case kAudioLevelControlPropertyDecibelRange:
            *outSize = sizeof(AudioValueRange); return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_volume_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, UInt32* outDataSize, void* outData) {
    Mirror* m = &gMirrors[idx];
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
            WRITE(AudioClassID, kAudioControlClassID);
        case kAudioObjectPropertyClass:
            WRITE(AudioClassID, kAudioVolumeControlClassID);
        case kAudioObjectPropertyOwner:
            WRITE(AudioObjectID, dev_id(idx));
        case kAudioObjectPropertyOwnedObjects:
            if (outDataSize) *outDataSize = 0; return noErr;
        case kAudioControlPropertyScope:
            WRITE(AudioObjectPropertyScope, kAudioObjectPropertyScopeOutput);
        case kAudioControlPropertyElement:
            WRITE(AudioObjectPropertyElement, kAudioObjectPropertyElementMain);
        case kAudioLevelControlPropertyScalarValue: {
            Float32 v = atomic_load_explicit(&m->volume, memory_order_relaxed);
            WRITE(Float32, v);
        }
        case kAudioLevelControlPropertyDecibelValue: {
            Float32 v = atomic_load_explicit(&m->volume, memory_order_relaxed);
            WRITE(Float32, scalar_to_db(v));
        }
        case kAudioLevelControlPropertyDecibelRange: {
            if (outDataSize) *outDataSize = sizeof(AudioValueRange);
            if (dataSize >= sizeof(AudioValueRange)) {
                AudioValueRange* r = (AudioValueRange*)outData;
                r->mMinimum = kMinDB; r->mMaximum = 0.0f;
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
        case kAudioLevelControlPropertyConvertScalarToDecibels: {
            if (dataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            Float32* v = (Float32*)outData;
            *v = scalar_to_db(*v);
            if (outDataSize) *outDataSize = sizeof(Float32);
            return noErr;
        }
        case kAudioLevelControlPropertyConvertDecibelsToScalar: {
            if (dataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            Float32* v = (Float32*)outData;
            *v = db_to_scalar(*v);
            if (outDataSize) *outDataSize = sizeof(Float32);
            return noErr;
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

static void notify_volume_changed(int idx) {
    if (!gHost) return;
    static const AudioObjectPropertyAddress changed[2] = {
        { kAudioLevelControlPropertyScalarValue,
          kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
        { kAudioLevelControlPropertyDecibelValue,
          kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
    };
    gHost->PropertiesChanged(gHost, volc_id(idx), 2, changed);
}

static OSStatus set_volume_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, const void* data) {
    Mirror* m = &gMirrors[idx];
    switch (a->mSelector) {
        case kAudioLevelControlPropertyScalarValue: {
            if (dataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            Float32 v = *(const Float32*)data;
            if (!(v >= 0.0f)) v = 0.0f; else if (v > 1.0f) v = 1.0f;
            atomic_store_explicit(&m->volume, v, memory_order_relaxed);
            notify_volume_changed(idx);
            return noErr;
        }
        case kAudioLevelControlPropertyDecibelValue: {
            if (dataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            Float32 v = db_to_scalar(*(const Float32*)data);
            atomic_store_explicit(&m->volume, v, memory_order_relaxed);
            notify_volume_changed(idx);
            return noErr;
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Mute control
// ===========================================================================
static OSStatus get_mute_prop_size(const AudioObjectPropertyAddress* a, UInt32* outSize) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyOwnedObjects:
            *outSize = 0; return noErr;
        case kAudioControlPropertyScope:
            *outSize = sizeof(AudioObjectPropertyScope); return noErr;
        case kAudioControlPropertyElement:
            *outSize = sizeof(AudioObjectPropertyElement); return noErr;
        case kAudioBooleanControlPropertyValue:
            *outSize = sizeof(UInt32); return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_mute_prop(int idx, const AudioObjectPropertyAddress* a,
                              UInt32 dataSize, UInt32* outDataSize, void* outData) {
    Mirror* m = &gMirrors[idx];
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
            WRITE(AudioClassID, kAudioControlClassID);
        case kAudioObjectPropertyClass:
            WRITE(AudioClassID, kAudioMuteControlClassID);
        case kAudioObjectPropertyOwner:
            WRITE(AudioObjectID, dev_id(idx));
        case kAudioObjectPropertyOwnedObjects:
            if (outDataSize) *outDataSize = 0; return noErr;
        case kAudioControlPropertyScope:
            WRITE(AudioObjectPropertyScope, kAudioObjectPropertyScopeOutput);
        case kAudioControlPropertyElement:
            WRITE(AudioObjectPropertyElement, kAudioObjectPropertyElementMain);
        case kAudioBooleanControlPropertyValue: {
            UInt32 v = atomic_load_explicit(&m->muted, memory_order_relaxed) ? 1 : 0;
            WRITE(UInt32, v);
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus set_mute_prop(int idx, const AudioObjectPropertyAddress* a,
                              UInt32 dataSize, const void* data) {
    Mirror* m = &gMirrors[idx];
    if (a->mSelector == kAudioBooleanControlPropertyValue) {
        if (dataSize < sizeof(UInt32)) return kAudioHardwareBadPropertySizeError;
        atomic_store_explicit(&m->muted, *(const UInt32*)data != 0, memory_order_relaxed);
        if (gHost) {
            AudioObjectPropertyAddress changed[1] = {
                { kAudioBooleanControlPropertyValue,
                  kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain }
            };
            gHost->PropertiesChanged(gHost, mute_id(idx), 1, changed);
        }
        return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Top-level dispatch
// ===========================================================================
static Boolean VM_HasProperty(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t c,
                              const AudioObjectPropertyAddress* a) {
    UInt32 dummy;
    return VM_GetPropertyDataSize(d, o, c, a, 0, NULL, &dummy) == noErr;
}

static OSStatus VM_IsPropertySettable(AudioServerPlugInDriverRef d, AudioObjectID o,
                                      pid_t c, const AudioObjectPropertyAddress* a,
                                      Boolean* out) {
    (void)d; (void)c;
    if (out == NULL) return kAudioHardwareIllegalOperationError;
    *out = false;
    int idx = object_to_mirror(o);
    if (idx >= 0) {
        if (o == volc_id(idx) &&
            (a->mSelector == kAudioLevelControlPropertyScalarValue ||
             a->mSelector == kAudioLevelControlPropertyDecibelValue)) *out = true;
        else if (o == mute_id(idx) && a->mSelector == kAudioBooleanControlPropertyValue) *out = true;
        else if (o == dev_id(idx) &&
                 (a->mSelector == kVMProperty_FollowTarget ||
                  a->mSelector == kVMProperty_FollowName)) *out = true;
    }
    return noErr;
}

static OSStatus VM_GetPropertyDataSize(AudioServerPlugInDriverRef d, AudioObjectID o,
                                       pid_t c, const AudioObjectPropertyAddress* a,
                                       UInt32 qs, const void* q, UInt32* outSize) {
    (void)d; (void)c; (void)qs; (void)q;
    if (a == NULL || outSize == NULL) return kAudioHardwareIllegalOperationError;
    if (o == kPlugInObjectID) return get_plugin_prop_size(a, outSize);
    int idx = object_to_mirror(o);
    if (idx < 0) return kAudioHardwareBadObjectError;
    if (o == dev_id(idx))                                       return get_device_prop_size(idx, a, outSize);
    if (o == outstrm_id(idx) || o == instrm_id(idx))            return get_stream_prop_size(a, outSize);
    if (o == volc_id(idx))                                      return get_volume_prop_size(a, outSize);
    if (o == mute_id(idx))                                      return get_mute_prop_size(a, outSize);
    return kAudioHardwareBadObjectError;
}

static OSStatus VM_GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
                                   pid_t c, const AudioObjectPropertyAddress* a,
                                   UInt32 qs, const void* q,
                                   UInt32 dataSize, UInt32* outDataSize, void* outData) {
    (void)d; (void)c; (void)qs;
    if (a == NULL || outData == NULL) return kAudioHardwareIllegalOperationError;
    if (o == kPlugInObjectID) return get_plugin_prop(a, dataSize, q, outDataSize, outData);
    int idx = object_to_mirror(o);
    if (idx < 0) return kAudioHardwareBadObjectError;
    if (o == dev_id(idx))                                       return get_device_prop(idx, a, dataSize, outDataSize, outData);
    if (o == outstrm_id(idx) || o == instrm_id(idx))            return get_stream_prop(idx, o, a, dataSize, outDataSize, outData);
    if (o == volc_id(idx))                                      return get_volume_prop(idx, a, dataSize, outDataSize, outData);
    if (o == mute_id(idx))                                      return get_mute_prop(idx, a, dataSize, outDataSize, outData);
    return kAudioHardwareBadObjectError;
}

static OSStatus VM_SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
                                   pid_t c, const AudioObjectPropertyAddress* a,
                                   UInt32 qs, const void* q,
                                   UInt32 dataSize, const void* data) {
    (void)d; (void)c; (void)qs; (void)q;
    if (a == NULL) return kAudioHardwareIllegalOperationError;
    int idx = object_to_mirror(o);
    if (idx < 0) return kAudioHardwareBadObjectError;
    if (o == volc_id(idx)) return set_volume_prop(idx, a, dataSize, data);
    if (o == mute_id(idx)) return set_mute_prop(idx, a, dataSize, data);
    if (o == dev_id(idx))  return set_device_prop(idx, a, dataSize, data);
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// IO
// ===========================================================================
static OSStatus VM_StartIO(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 c) {
    (void)d; (void)c;
    int idx = object_to_mirror(id);
    if (idx < 0 || id != dev_id(idx)) return kAudioHardwareBadObjectError;
    Mirror* m = &gMirrors[idx];

    pthread_mutex_lock(&m->mu);
    UInt32 prev = atomic_load_explicit(&m->ioRunning, memory_order_relaxed);
    if (prev == UINT32_MAX) { pthread_mutex_unlock(&m->mu); return kAudioHardwareIllegalOperationError; }
    if (prev > 0) {
        atomic_store_explicit(&m->ioRunning, prev + 1, memory_order_release);
        pthread_mutex_unlock(&m->mu);
        return noErr;
    }
    // 0 → 1: reset ring + anchor host time.
    ring_reset(&m->ring);
    m->currentGain = 0.0f;
    m->ioStartHost = mach_absolute_time();
    atomic_store_explicit(&m->ioRunning, 1, memory_order_release);
    pthread_mutex_unlock(&m->mu);
    VMLOG("StartIO: slot %{public}d", idx);
    return noErr;
}

static OSStatus VM_StopIO(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 c) {
    (void)d; (void)c;
    int idx = object_to_mirror(id);
    if (idx < 0 || id != dev_id(idx)) return kAudioHardwareBadObjectError;
    Mirror* m = &gMirrors[idx];

    pthread_mutex_lock(&m->mu);
    UInt32 prev = atomic_load_explicit(&m->ioRunning, memory_order_relaxed);
    if (prev == 0)  { pthread_mutex_unlock(&m->mu); return noErr; }
    if (prev > 1)   {
        atomic_store_explicit(&m->ioRunning, prev - 1, memory_order_release);
        pthread_mutex_unlock(&m->mu);
        return noErr;
    }
    atomic_store_explicit(&m->ioRunning, 0, memory_order_release);
    pthread_mutex_unlock(&m->mu);
    VMLOG("StopIO: slot %{public}d", idx);
    return noErr;
}

// Free-running clock anchored at StartIO. The agent's IOProc on the input
// stream paces sample-by-sample reading via the ring's natural rate.
static OSStatus VM_GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID id,
                                    UInt32 c, Float64* outSampleTime,
                                    UInt64* outHostTime, UInt64* outSeed) {
    (void)d; (void)c;
    int idx = object_to_mirror(id);
    if (idx < 0) return kAudioHardwareBadObjectError;
    Mirror* m = &gMirrors[idx];

    UInt64 startHost = m->ioStartHost;
    UInt64 now = mach_absolute_time();
    UInt64 elapsedTicks = now > startHost ? now - startHost : 0;
    Float64 elapsedNs = (Float64)elapsedTicks * (Float64)gTb.numer / (Float64)gTb.denom;
    Float64 elapsedSamples = elapsedNs * m->sampleRate / 1.0e9;
    UInt64 cycles = (UInt64)(elapsedSamples / (Float64)kZeroTimeStampPeriod);
    UInt64 sampleTime = cycles * kZeroTimeStampPeriod;
    Float64 hostTicksPerCycle = (Float64)kZeroTimeStampPeriod * 1.0e9 / m->sampleRate
                                * (Float64)gTb.denom / (Float64)gTb.numer;
    UInt64 hostTime = startHost + (UInt64)((Float64)cycles * hostTicksPerCycle);

    if (outSampleTime) *outSampleTime = (Float64)sampleTime;
    if (outHostTime)   *outHostTime   = hostTime;
    if (outSeed)       *outSeed       = 1;
    return noErr;
}

static OSStatus VM_WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id,
                                     UInt32 c, UInt32 op, Boolean* w, Boolean* wp) {
    (void)d; (void)id; (void)c;
    Boolean want = (op == kAudioServerPlugInIOOperationWriteMix ||
                    op == kAudioServerPlugInIOOperationReadInput);
    if (w)  *w  = want;
    if (wp) *wp = want;
    return noErr;
}

static OSStatus VM_BeginIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id,
                                    UInt32 c, UInt32 op, UInt32 fs,
                                    const AudioServerPlugInIOCycleInfo* ci) {
    (void)d; (void)id; (void)c; (void)op; (void)fs; (void)ci;
    return noErr;
}

static OSStatus VM_DoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id,
                                 AudioObjectID s, UInt32 c, UInt32 op, UInt32 fs,
                                 const AudioServerPlugInIOCycleInfo* ci,
                                 void* mainBuffer, void* secondaryBuffer) {
    (void)d; (void)c; (void)ci; (void)secondaryBuffer;
    if (!mainBuffer || fs == 0) return noErr;
    int idx = object_to_mirror(id);
    if (idx < 0) return noErr;
    Mirror* m = &gMirrors[idx];
    if (atomic_load_explicit(&m->ioRunning, memory_order_acquire) == 0) return noErr;

    if (op == kAudioServerPlugInIOOperationWriteMix && s == outstrm_id(idx)) {
        float scalar     = atomic_load_explicit(&m->volume, memory_order_relaxed);
        bool  muted      = atomic_load_explicit(&m->muted,  memory_order_relaxed);
        float targetGain = muted ? 0.0f : scalar_to_gain(scalar);
        float startGain  = m->currentGain;
        ring_write_scaled(&m->ring, (const float*)mainBuffer, fs, startGain, targetGain);
        m->currentGain = targetGain;
        return noErr;
    }
    if (op == kAudioServerPlugInIOOperationReadInput && s == instrm_id(idx)) {
        ring_read_stereo(&m->ring, (float*)mainBuffer, fs);
        return noErr;
    }
    return noErr;
}

static OSStatus VM_EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id,
                                  UInt32 c, UInt32 op, UInt32 fs,
                                  const AudioServerPlugInIOCycleInfo* ci) {
    (void)d; (void)id; (void)c; (void)op; (void)fs; (void)ci;
    return noErr;
}

// ===========================================================================
// Lifecycle
// ===========================================================================
static OSStatus VM_Initialize(AudioServerPlugInDriverRef d, AudioServerPlugInHostRef h) {
    (void)d;
    VMLOG("VM_Initialize");
    gHost = h;
    mach_timebase_info(&gTb);
    for (int i = 0; i < kMaxMirrors; i++) {
        Mirror* m = &gMirrors[i];
        pthread_mutex_init(&m->mu, NULL);
        ring_init(&m->ring, kRingCapacity);
        m->realUID[0] = 0;
        snprintf(m->displayName, sizeof(m->displayName), "VolMirror Slot %d", i);
        slot_resync(m, i);
        m->sampleRate = kDefaultSampleRate;
        atomic_init(&m->volume, 1.0f);
        atomic_init(&m->muted, false);
        atomic_init(&m->ioRunning, 0);
        m->currentGain = 0.0f;
        m->ioStartHost = 0;
    }
    return noErr;
}

static OSStatus VM_CreateDevice(AudioServerPlugInDriverRef d, CFDictionaryRef desc,
                                const AudioServerPlugInClientInfo* info,
                                AudioObjectID* outID) {
    (void)d; (void)desc; (void)info; (void)outID;
    return kAudioHardwareUnsupportedOperationError;
}
static OSStatus VM_DestroyDevice(AudioServerPlugInDriverRef d, AudioObjectID id) {
    (void)d; (void)id;
    return kAudioHardwareUnsupportedOperationError;
}
static OSStatus VM_AddDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID id,
                                   const AudioServerPlugInClientInfo* info) {
    (void)d; (void)id; (void)info; return noErr;
}
static OSStatus VM_RemoveDeviceClient(AudioServerPlugInDriverRef d, AudioObjectID id,
                                      const AudioServerPlugInClientInfo* info) {
    (void)d; (void)id; (void)info; return noErr;
}
static OSStatus VM_PerformDeviceConfigChange(AudioServerPlugInDriverRef d,
                                             AudioObjectID id, UInt64 act, void* i) {
    (void)d; (void)id; (void)act; (void)i; return noErr;
}
static OSStatus VM_AbortDeviceConfigChange(AudioServerPlugInDriverRef d,
                                           AudioObjectID id, UInt64 act, void* i) {
    (void)d; (void)id; (void)act; (void)i; return noErr;
}

// ===========================================================================
// IUnknown + factory
// ===========================================================================
__attribute__((visibility("default")))
void* VolMirror_Factory(CFAllocatorRef allocator, CFUUIDRef typeUUID);

void* VolMirror_Factory(CFAllocatorRef allocator, CFUUIDRef typeUUID) {
    (void)allocator;
    if (typeUUID == NULL) return NULL;
    if (!CFEqual(typeUUID, kAudioServerPlugInTypeUUID)) return NULL;
    return &gInterfacePtr;
}

static HRESULT VM_QueryInterface(void* self, REFIID iid, LPVOID* outIface) {
    (void)self;
    if (outIface == NULL) return kAudioHardwareIllegalOperationError;
    CFUUIDRef requested = CFUUIDCreateFromUUIDBytes(NULL, iid);
    if (!requested) return kAudioHardwareIllegalOperationError;
    HRESULT r = E_NOINTERFACE;
    if (CFEqual(requested, IUnknownUUID) ||
        CFEqual(requested, kAudioServerPlugInDriverInterfaceUUID)) {
        atomic_fetch_add(&gRefCount, 1);
        *outIface = &gInterfacePtr;
        r = 0;
    }
    CFRelease(requested);
    return r;
}
static ULONG VM_AddRef(void* self) {
    (void)self;
    return atomic_fetch_add(&gRefCount, 1) + 1;
}
static ULONG VM_Release(void* self) {
    (void)self;
    unsigned cur = atomic_load_explicit(&gRefCount, memory_order_relaxed);
    while (cur > 0 && !atomic_compare_exchange_weak_explicit(
               &gRefCount, &cur, cur - 1,
               memory_order_acq_rel, memory_order_relaxed)) { }
    return cur > 0 ? cur - 1 : 0;
}
