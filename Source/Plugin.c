// VolMirror — CoreAudio Server Plugin
//
// Iteration 2: enumerate output devices with locked hardware volume and
// publish a paired "(Vol)" virtual device per match. Each mirror exposes a
// settable volume scalar (so the OS slider / F11 / F12 work natively).
// IO is stubbed — the device is selectable and doesn't crash, but produces
// silence until iteration 3 wires the ring buffer + forwarder.

#include <CoreAudio/AudioServerPlugIn.h>
#include <CoreAudio/AudioHardware.h>
#include <CoreFoundation/CFPlugInCOM.h>
#include <CoreFoundation/CoreFoundation.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>
#include <os/log.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define VMLOG(fmt, ...) os_log(OS_LOG_DEFAULT, "[VolMirror] " fmt, ##__VA_ARGS__)
// Set to 1 to log every property call (verbose). Off by default.
#ifndef VOLMIRROR_LOG_PROPERTIES
#define VOLMIRROR_LOG_PROPERTIES 0
#endif
#if VOLMIRROR_LOG_PROPERTIES
#define VMLOG_PROP(...) VMLOG(__VA_ARGS__)
#else
#define VMLOG_PROP(...) ((void)0)
#endif

// ===========================================================================
// Object ID layout
// ===========================================================================
#define kPlugInObjectID         kAudioObjectPlugInObject  // == 1, mandated by AudioServerPlugIn.h
#define kMaxMirrors             8
#define kFirstMirrorObjectID    ((AudioObjectID)100)

#define kFallbackSampleRate     48000.0  // used only when the real device's rate query fails
#define kRingFrames             1024     // ZeroTimeStampPeriod

static inline AudioObjectID dev_id(int i) { return kFirstMirrorObjectID + i*10 + 0; }
static inline AudioObjectID strm_id(int i){ return kFirstMirrorObjectID + i*10 + 1; }
static inline AudioObjectID volc_id(int i){ return kFirstMirrorObjectID + i*10 + 2; }
static inline AudioObjectID mute_id(int i){ return kFirstMirrorObjectID + i*10 + 3; }

static int object_to_mirror(AudioObjectID o) {
    if (o < kFirstMirrorObjectID) return -1;
    int i = (int)((o - kFirstMirrorObjectID) / 10);
    if (i < 0 || i >= kMaxMirrors) return -1;
    return i;
}

// ===========================================================================
// SPSC ring buffer (stereo float32, interleaved). Power-of-two capacity in frames.
// ===========================================================================
#define kRingCapacity 4096  // ~85 ms at 48 kHz

typedef struct {
    float*       data;          // capacity * 2 floats
    UInt32       capacity;      // frames, power of 2
    atomic_uint  head;          // consumer position
    atomic_uint  tail;          // producer position
} Ring;

// ===========================================================================
// Mirror state
// ===========================================================================
typedef struct {
    _Atomic bool        active;             // flipped under gListMu, read lock-free elsewhere
    AudioObjectID       realID;             // paired real device (changes across hotplug)
    char                realUID[224];       // real device's stable UID (for matching)
    char                baseName[256];      // "LG ULTRAWIDE"
    char                displayName[300];   // "LG ULTRAWIDE (Vol)"
    char                uid[260];           // our device UID = "VolMirror/<realUID>"
    char                modelUID[160];
    Float64             sampleRate;         // captured at reconcile-add from real device; fixed for slot lifetime
    _Atomic float       volume;             // 0..1, written by OS slider, read by RT thread
    _Atomic bool        muted;              // OS mute (F10 / menu bar) — read on RT path
    float               currentGain;        // last applied gain — producer-thread only, used for per-cycle ramp
    _Atomic bool        ioRunning;          // read by RT WriteMix without locking
    _Atomic UInt64      ioStartHost;        // host-time fallback anchor (used only until first real_io_proc fires)
    Ring                ring;               // virtual-IOProc → real-IOProc
    AudioDeviceIOProcID realProc;           // our IOProc on the paired real device
    pthread_mutex_t     mu;                 // guards lifecycle (StartIO/StopIO)

    // Drift correction. real_io_proc (running on the real device's RT thread)
    // captures the real device's IO timing into a seqlock-protected pair;
    // VM_GetZeroTimeStamp reads it. Anchoring our virtual-device timeline to
    // the real device's clock makes coreaudiod pace our producer at the rate
    // the consumer actually drains the ring, eliminating long-run drift.
    _Atomic UInt64      driftSeq;           // seqlock: even=stable, odd=writing
    UInt64              driftSampleTime;    // virtual-device sample time, cycle-aligned
    UInt64              driftHostTime;      // host time at driftSampleTime
    _Atomic bool        driftPrimed;        // false until first real_io_proc capture
    Float64             realRefSample;      // real device's sample time at first capture
    _Atomic UInt64      driftSeed;          // bumped on StartIO so coreaudiod re-anchors
} Mirror;

static Mirror                       gMirrors[kMaxMirrors];
static AudioServerPlugInHostRef     gHost = NULL;
static atomic_uint                  gRefCount = 0;
static pthread_mutex_t              gListMu = PTHREAD_MUTEX_INITIALIZER;
static mach_timebase_info_data_t    gTb;
static dispatch_queue_t             gReconcileQ = NULL;     // serial
static dispatch_source_t            gReconcileTimer = NULL; // debounce: collapses bursty hotplug events

// Forward decl — defined alongside reconcile_devices.
static void schedule_reconcile(void);

// Stereo Float32 interleaved at the mirror's per-slot sample rate (matches the real device).
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

// ===========================================================================
// V-table forward declarations
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
// Helpers for property responses
// ===========================================================================
#define WRITE(type, val)                                              \
    do {                                                              \
        if (outDataSize) *outDataSize = sizeof(type);                 \
        if (dataSize >= sizeof(type)) *(type*)outData = (val);        \
        else return kAudioHardwareBadPropertySizeError;               \
        return noErr;                                                 \
    } while (0)

#define WRITE_CFSTR(s)                                                \
    do {                                                              \
        if (outDataSize) *outDataSize = sizeof(CFStringRef);          \
        if (dataSize >= sizeof(CFStringRef))                          \
            *(CFStringRef*)outData = CFStringCreateCopy(NULL, (s));   \
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
// Property handling — plugin object
// ===========================================================================
static Boolean has_plugin_prop(const AudioObjectPropertyAddress* a) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyManufacturer:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyBoxList:
        case kAudioPlugInPropertyTranslateUIDToBox:
        case kAudioPlugInPropertyDeviceList:
        case kAudioPlugInPropertyTranslateUIDToDevice:
        case kAudioPlugInPropertyResourceBundle:
            return true;
    }
    return false;
}

static OSStatus get_plugin_prop_size(const AudioObjectPropertyAddress* a, UInt32* outSize) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
            *outSize = sizeof(AudioObjectID); return noErr;
        case kAudioObjectPropertyManufacturer:
        case kAudioPlugInPropertyResourceBundle:
            *outSize = sizeof(CFStringRef); return noErr;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList:
        case kAudioPlugInPropertyBoxList: {
            int n = 0;
            pthread_mutex_lock(&gListMu);
            for (int i = 0; i < kMaxMirrors; i++)
                if (atomic_load_explicit(&gMirrors[i].active, memory_order_acquire)) n++;
            pthread_mutex_unlock(&gListMu);
            // BoxList is empty; OwnedObjects + DeviceList contain the mirror devices.
            if (a->mSelector == kAudioPlugInPropertyBoxList) {
                *outSize = 0;
            } else {
                *outSize = (UInt32)(n * sizeof(AudioObjectID));
            }
            return noErr;
        }
        case kAudioPlugInPropertyTranslateUIDToBox:
        case kAudioPlugInPropertyTranslateUIDToDevice:
            *outSize = sizeof(AudioObjectID); return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus get_plugin_prop(const AudioObjectPropertyAddress* a, UInt32 dataSize,
                                const void* qual, UInt32* outDataSize, void* outData,
                                UInt32 qualSize) {
    (void)qualSize;
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
            WRITE_CFSTR(CFSTR(""));
        case kAudioPlugInPropertyBoxList:
            if (outDataSize) *outDataSize = 0;
            return noErr;
        case kAudioObjectPropertyOwnedObjects:
        case kAudioPlugInPropertyDeviceList: {
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            UInt32 n = 0;
            AudioObjectID* dst = (AudioObjectID*)outData;
            pthread_mutex_lock(&gListMu);
            for (int i = 0; i < kMaxMirrors && n < cap; i++)
                if (atomic_load_explicit(&gMirrors[i].active, memory_order_acquire))
                    dst[n++] = dev_id(i);
            pthread_mutex_unlock(&gListMu);
            if (outDataSize) *outDataSize = (UInt32)(n * sizeof(AudioObjectID));
            return noErr;
        }
        case kAudioPlugInPropertyTranslateUIDToBox:
            WRITE(AudioObjectID, kAudioObjectUnknown);
        case kAudioPlugInPropertyTranslateUIDToDevice: {
            if (qual == NULL) return kAudioHardwareBadPropertySizeError;
            CFStringRef in = *(CFStringRef*)qual;
            AudioObjectID found = kAudioObjectUnknown;
            pthread_mutex_lock(&gListMu);
            for (int i = 0; i < kMaxMirrors; i++) {
                if (!atomic_load_explicit(&gMirrors[i].active, memory_order_acquire)) continue;
                CFStringRef mine = CFStringCreateWithCString(NULL, gMirrors[i].uid,
                                                              kCFStringEncodingUTF8);
                if (CFStringCompare(in, mine, 0) == kCFCompareEqualTo) {
                    found = dev_id(i);
                    CFRelease(mine);
                    break;
                }
                CFRelease(mine);
            }
            pthread_mutex_unlock(&gListMu);
            WRITE(AudioObjectID, found);
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Property handling — device
// ===========================================================================
static UInt32 streams_for_scope(int idx, AudioObjectPropertyScope scope,
                                AudioObjectID* out, UInt32 cap) {
    (void)idx;
    UInt32 n = 0;
    if (scope == kAudioObjectPropertyScopeOutput || scope == kAudioObjectPropertyScopeGlobal) {
        if (cap > n) out[n] = strm_id(idx);
        n++;
    }
    return n;
}

static Boolean has_device_prop(int idx, const AudioObjectPropertyAddress* a) {
    (void)idx;
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyName:
        case kAudioObjectPropertyManufacturer:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioDevicePropertyDeviceUID:
        case kAudioDevicePropertyModelUID:
        case kAudioDevicePropertyTransportType:
        case kAudioDevicePropertyRelatedDevices:
        case kAudioDevicePropertyClockDomain:
        case kAudioDevicePropertyDeviceIsAlive:
        case kAudioDevicePropertyDeviceIsRunning:
        case kAudioObjectPropertyControlList:
        case kAudioDevicePropertyNominalSampleRate:
        case kAudioDevicePropertyAvailableNominalSampleRates:
        case kAudioDevicePropertyIsHidden:
        case kAudioDevicePropertyZeroTimeStampPeriod:
        case kAudioDevicePropertyStreams:
        case kAudioDevicePropertyLatency:
        case kAudioDevicePropertySafetyOffset:
        case kAudioDevicePropertyDeviceCanBeDefaultDevice:
        case kAudioDevicePropertyDeviceCanBeDefaultSystemDevice:
        case kAudioDevicePropertyPreferredChannelsForStereo:
            return true;
    }
    return false;
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
        case kAudioObjectPropertyOwnedObjects:
        case kAudioObjectPropertyControlList: {
            // ControlList: volume + mute. OwnedObjects: stream + volume + mute
            // (global/output scope). Input scope owns nothing.
            if (a->mSelector == kAudioObjectPropertyControlList) {
                *outSize = 2 * sizeof(AudioObjectID);
            } else if (a->mScope == kAudioObjectPropertyScopeInput) {
                *outSize = 0;
            } else {
                *outSize = 3 * sizeof(AudioObjectID);
            }
            return noErr;
        }
        case kAudioDevicePropertyStreams: {
            UInt32 n = streams_for_scope(idx, a->mScope, NULL, 0);
            *outSize = n * sizeof(AudioObjectID); return noErr;
        }
        case kAudioDevicePropertyRelatedDevices:
            *outSize = sizeof(AudioObjectID); return noErr;
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
        case kAudioObjectPropertyName:
            WRITE_CSTR(m->displayName);
        case kAudioObjectPropertyManufacturer:
            WRITE_CSTR("VolMirror");
        case kAudioDevicePropertyDeviceUID:
            WRITE_CSTR(m->uid);
        case kAudioDevicePropertyModelUID:
            WRITE_CSTR(m->modelUID);
        case kAudioDevicePropertyTransportType:
            WRITE(UInt32, kAudioDeviceTransportTypeVirtual);
        case kAudioDevicePropertyClockDomain:
            WRITE(UInt32, 0);
        case kAudioDevicePropertyDeviceIsAlive:
            WRITE(UInt32, 1);
        case kAudioDevicePropertyDeviceIsRunning:
            WRITE(UInt32, atomic_load_explicit(&m->ioRunning, memory_order_relaxed) ? 1 : 0);
        case kAudioDevicePropertyIsHidden:
            WRITE(UInt32, 0);
        case kAudioDevicePropertyZeroTimeStampPeriod:
            WRITE(UInt32, kRingFrames);
        case kAudioDevicePropertyLatency:
            // One IO cycle through the SPSC ring; the real device contributes its own
            // latency separately (read by clients via the realID).
            WRITE(UInt32, kRingFrames);
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
        case kAudioDevicePropertyPreferredChannelsForStereo: {
            if (outDataSize) *outDataSize = 2 * sizeof(UInt32);
            if (dataSize >= 2 * sizeof(UInt32)) {
                ((UInt32*)outData)[0] = 1;
                ((UInt32*)outData)[1] = 2;
                return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
        case kAudioObjectPropertyOwnedObjects:
        case kAudioObjectPropertyControlList: {
            if (a->mSelector == kAudioObjectPropertyControlList) {
                UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
                AudioObjectID* dst = (AudioObjectID*)outData;
                UInt32 n = 0;
                if (cap > 0) dst[n++] = volc_id(idx);
                if (cap > 1) dst[n++] = mute_id(idx);
                if (outDataSize) *outDataSize = n * sizeof(AudioObjectID);
                return noErr;
            }
            if (a->mScope == kAudioObjectPropertyScopeInput) {
                if (outDataSize) *outDataSize = 0; return noErr;
            }
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            AudioObjectID* dst = (AudioObjectID*)outData;
            UInt32 n = 0;
            if (cap > 0) dst[n++] = strm_id(idx);
            if (cap > 1) dst[n++] = volc_id(idx);
            if (cap > 2) dst[n++] = mute_id(idx);
            if (outDataSize) *outDataSize = n * sizeof(AudioObjectID);
            return noErr;
        }
        case kAudioDevicePropertyStreams: {
            UInt32 cap = dataSize / (UInt32)sizeof(AudioObjectID);
            UInt32 n = streams_for_scope(idx, a->mScope, (AudioObjectID*)outData, cap);
            if (outDataSize) *outDataSize = n * sizeof(AudioObjectID);
            return noErr;
        }
        case kAudioDevicePropertyRelatedDevices: {
            if (outDataSize) *outDataSize = sizeof(AudioObjectID);
            if (dataSize >= sizeof(AudioObjectID)) {
                *(AudioObjectID*)outData = dev_id(idx); return noErr;
            }
            return kAudioHardwareBadPropertySizeError;
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

static OSStatus set_device_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, const void* data) {
    (void)idx; (void)a; (void)dataSize; (void)data;
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Property handling — stream
// ===========================================================================
static Boolean has_stream_prop(const AudioObjectPropertyAddress* a) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioStreamPropertyIsActive:
        case kAudioStreamPropertyDirection:
        case kAudioStreamPropertyTerminalType:
        case kAudioStreamPropertyStartingChannel:
        case kAudioStreamPropertyLatency:
        case kAudioStreamPropertyVirtualFormat:
        case kAudioStreamPropertyAvailableVirtualFormats:
        case kAudioStreamPropertyPhysicalFormat:
        case kAudioStreamPropertyAvailablePhysicalFormats:
            return true;
    }
    return false;
}

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

static OSStatus get_stream_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, UInt32* outDataSize, void* outData) {
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
            WRITE(UInt32, 0); // 0 = output
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
// Property handling — volume control
// ===========================================================================
static Boolean has_volume_prop(const AudioObjectPropertyAddress* a) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioControlPropertyScope:
        case kAudioControlPropertyElement:
        case kAudioLevelControlPropertyScalarValue:
        case kAudioLevelControlPropertyDecibelValue:
        case kAudioLevelControlPropertyDecibelRange:
        case kAudioLevelControlPropertyConvertScalarToDecibels:
        case kAudioLevelControlPropertyConvertDecibelsToScalar:
            return true;
    }
    return false;
}

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

static const Float32 kMinDB = -96.0f;

// Quadratic taper: gain = scalar^2. Matching dB curve: 20·log10(s^2) =
// 40·log10(s). Steeper than linear (so high-end of the slider isn't packed
// into the last few percent), shallower than cubic (so the low end stays
// audible instead of dropping to silence).
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
    // Negated comparison so NaN clamps to silence instead of leaking through.
    if (!(d > kMinDB)) return 0.0f;
    if (d >= 0.0f)    return 1.0f;
    return powf(10.0f, d / 40.0f);
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

static OSStatus set_volume_prop(int idx, const AudioObjectPropertyAddress* a,
                                UInt32 dataSize, const void* data) {
    Mirror* m = &gMirrors[idx];
    switch (a->mSelector) {
        case kAudioLevelControlPropertyScalarValue: {
            if (dataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            Float32 v = *(const Float32*)data;
            // Negated comparisons so NaN falls into the clamp branch.
            if (!(v >= 0.0f)) v = 0.0f; else if (v > 1.0f) v = 1.0f;
            atomic_store_explicit(&m->volume, v, memory_order_relaxed);
            // Notify the host that scalar + dB both changed.
            AudioObjectPropertyAddress changed[2] = {
                { kAudioLevelControlPropertyScalarValue,
                  kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
                { kAudioLevelControlPropertyDecibelValue,
                  kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
            };
            if (gHost) gHost->PropertiesChanged(gHost, volc_id(idx), 2, changed);
            return noErr;
        }
        case kAudioLevelControlPropertyDecibelValue: {
            if (dataSize < sizeof(Float32)) return kAudioHardwareBadPropertySizeError;
            Float32 v = db_to_scalar(*(const Float32*)data);
            atomic_store_explicit(&m->volume, v, memory_order_relaxed);
            AudioObjectPropertyAddress changed[2] = {
                { kAudioLevelControlPropertyScalarValue,
                  kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
                { kAudioLevelControlPropertyDecibelValue,
                  kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain },
            };
            if (gHost) gHost->PropertiesChanged(gHost, volc_id(idx), 2, changed);
            return noErr;
        }
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Property handling — mute control
// ===========================================================================
static Boolean has_mute_prop(const AudioObjectPropertyAddress* a) {
    switch (a->mSelector) {
        case kAudioObjectPropertyBaseClass:
        case kAudioObjectPropertyClass:
        case kAudioObjectPropertyOwner:
        case kAudioObjectPropertyOwnedObjects:
        case kAudioControlPropertyScope:
        case kAudioControlPropertyElement:
        case kAudioBooleanControlPropertyValue:
            return true;
    }
    return false;
}

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
        UInt32 v = *(const UInt32*)data;
        bool muted = (v != 0);
        atomic_store_explicit(&m->muted, muted, memory_order_relaxed);
        AudioObjectPropertyAddress changed[1] = {
            { kAudioBooleanControlPropertyValue,
              kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain }
        };
        if (gHost) gHost->PropertiesChanged(gHost, mute_id(idx), 1, changed);
        return noErr;
    }
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// Top-level property dispatch
// ===========================================================================
// Forward decl — used by real_is_output_with_locked_volume below.
static void copy_device_uid(AudioObjectID dev, char* dst, size_t dstSize);

__attribute__((unused))
static const char* fourcc_str(UInt32 c, char* buf) {
    buf[0] = (char)(c >> 24); buf[1] = (char)(c >> 16);
    buf[2] = (char)(c >> 8);  buf[3] = (char)c; buf[4] = 0;
    for (int i = 0; i < 4; i++) if (buf[i] < 32 || buf[i] > 126) buf[i] = '?';
    return buf;
}

static Boolean VM_HasProperty(AudioServerPlugInDriverRef d, AudioObjectID o, pid_t c,
                              const AudioObjectPropertyAddress* a) {
    (void)d; (void)c;
    if (a == NULL) return false;
    Boolean r = false;
    if (o == kPlugInObjectID) r = has_plugin_prop(a);
    else {
        int idx = object_to_mirror(o);
        if (idx >= 0 && atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire)) {
            if (o == dev_id(idx))       r = has_device_prop(idx, a);
            else if (o == strm_id(idx)) r = has_stream_prop(a);
            else if (o == volc_id(idx)) r = has_volume_prop(a);
            else if (o == mute_id(idx)) r = has_mute_prop(a);
        }
    }
#if VOLMIRROR_LOG_PROPERTIES
    char fc[5];
    VMLOG_PROP("HasProperty(obj=%{public}u sel=%{public}s scope=%{public}s) -> %{public}d",
          (unsigned)o, fourcc_str(a->mSelector, fc),
          a->mScope == kAudioObjectPropertyScopeOutput ? "out" :
          a->mScope == kAudioObjectPropertyScopeInput ? "in" : "glo",
          (int)r);
#endif
    return r;
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
             a->mSelector == kAudioLevelControlPropertyDecibelValue)) {
            *out = true;
        } else if (o == mute_id(idx) &&
                   a->mSelector == kAudioBooleanControlPropertyValue) {
            *out = true;
        }
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
    if (idx < 0 || !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
        return kAudioHardwareBadObjectError;
    if (o == dev_id(idx))  return get_device_prop_size(idx, a, outSize);
    if (o == strm_id(idx)) return get_stream_prop_size(a, outSize);
    if (o == volc_id(idx)) return get_volume_prop_size(a, outSize);
    if (o == mute_id(idx)) return get_mute_prop_size(a, outSize);
    return kAudioHardwareBadObjectError;
}

static OSStatus VM_GetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
                                   pid_t c, const AudioObjectPropertyAddress* a,
                                   UInt32 qs, const void* q,
                                   UInt32 dataSize, UInt32* outDataSize, void* outData) {
    (void)d; (void)c;
    OSStatus r;
    if (a == NULL || outData == NULL) r = kAudioHardwareIllegalOperationError;
    else if (o == kPlugInObjectID) r = get_plugin_prop(a, dataSize, q, outDataSize, outData, qs);
    else {
        int idx = object_to_mirror(o);
        if (idx < 0 || !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
            r = kAudioHardwareBadObjectError;
        else if (o == dev_id(idx))  r = get_device_prop(idx, a, dataSize, outDataSize, outData);
        else if (o == strm_id(idx)) r = get_stream_prop(idx, a, dataSize, outDataSize, outData);
        else if (o == volc_id(idx)) r = get_volume_prop(idx, a, dataSize, outDataSize, outData);
        else if (o == mute_id(idx)) r = get_mute_prop(idx, a, dataSize, outDataSize, outData);
        else r = kAudioHardwareBadObjectError;
    }
#if VOLMIRROR_LOG_PROPERTIES
    char fc[5];
    VMLOG_PROP("GetPropertyData(obj=%{public}u sel=%{public}s) -> %{public}d size=%{public}u",
          (unsigned)o, fourcc_str(a ? a->mSelector : 0, fc),
          (int)r, outDataSize ? *outDataSize : 0);
#endif
    return r;
}

static OSStatus VM_SetPropertyData(AudioServerPlugInDriverRef d, AudioObjectID o,
                                   pid_t c, const AudioObjectPropertyAddress* a,
                                   UInt32 qs, const void* q,
                                   UInt32 dataSize, const void* data) {
    (void)d; (void)c; (void)qs; (void)q;
    if (a == NULL) return kAudioHardwareIllegalOperationError;
    int idx = object_to_mirror(o);
    if (idx < 0 || !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
        return kAudioHardwareBadObjectError;
    if (o == volc_id(idx)) return set_volume_prop(idx, a, dataSize, data);
    if (o == mute_id(idx)) return set_mute_prop(idx, a, dataSize, data);
    if (o == dev_id(idx))  return set_device_prop(idx, a, dataSize, data);
    return kAudioHardwareUnknownPropertyError;
}

// ===========================================================================
// IO forwarding: virtual-IOProc (producer) → ring → real-device IOProc (consumer)
// ===========================================================================
// Allocated once per slot in VM_Initialize and never freed: the producer
// (VM_DoIOOperation) runs on coreaudiod's RT thread, which we cannot reliably
// quiesce from a hotplug path, so the buffer outlives every device transition.
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

// Producer side: write `frames` of stereo interleaved Float32, linearly ramping
// gain from gainStart to gainEnd across the buffer to avoid sample-boundary
// discontinuities (which a sudden gain step would produce as audible clicks
// during slider drag or mute toggle). Returns frames actually written (drops
// if ring is full — RT-safe, no blocking).
static UInt32 ring_write_scaled(Ring* r, const float* src, UInt32 frames,
                                float gainStart, float gainEnd) {
    if (!r->data) return 0;
    UInt32 mask = r->capacity - 1;
    UInt32 head = atomic_load_explicit(&r->head, memory_order_acquire);
    UInt32 tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    UInt32 used = (tail - head) & mask;
    UInt32 freeSpace = r->capacity - 1 - used;  // -1 to distinguish empty/full
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

// Consumer side: read up to `frames` and write into channels 0,1 of an interleaved
// destination buffer with given channel stride. Returns frames produced.
static UInt32 ring_read_to_stereo(Ring* r, float* dst, UInt32 channelStride, UInt32 frames) {
    if (!r->data) return 0;
    UInt32 mask = r->capacity - 1;
    UInt32 head = atomic_load_explicit(&r->head, memory_order_relaxed);
    UInt32 tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    UInt32 used = (tail - head) & mask;
    UInt32 toRead = frames < used ? frames : used;
    for (UInt32 i = 0; i < toRead; i++) {
        UInt32 idx = (head + i) & mask;
        dst[i * channelStride + 0] = r->data[idx*2 + 0];
        dst[i * channelStride + 1] = r->data[idx*2 + 1];
    }
    atomic_store_explicit(&r->head, (head + toRead) & mask, memory_order_release);
    return toRead;
}

// IOProc registered on the paired real device.
static OSStatus real_io_proc(AudioObjectID inDevice,
                             const AudioTimeStamp* inNow,
                             const AudioBufferList* inInputData,
                             const AudioTimeStamp* inInputTime,
                             AudioBufferList* outOutputData,
                             const AudioTimeStamp* inOutputTime,
                             void* clientData) {
    (void)inDevice; (void)inInputData; (void)inInputTime;
    Mirror* m = (Mirror*)clientData;
    if (!m || !outOutputData || outOutputData->mNumberBuffers == 0) return noErr;
    AudioBuffer* buf = &outOutputData->mBuffers[0];
    UInt32 channels = buf->mNumberChannels;
    if (channels < 2 || !buf->mData) return noErr;
    UInt32 bytesPerFrame = sizeof(float) * channels;
    UInt32 frames = buf->mDataByteSize / bytesPerFrame;
    float* out = (float*)buf->mData;
    // Zero the slot we own (so unwritten frames + extra channels are silent).
    memset(out, 0, frames * bytesPerFrame);
    ring_read_to_stereo(&m->ring, out, channels, frames);

    // Drift correction: publish the real device's clock to the seqlock so
    // VM_GetZeroTimeStamp can return real-anchored timestamps. Coreaudiod
    // uses those to pace our virtual-side IOProc at the real device's actual
    // rate, so the two clocks track each other instead of drifting apart.
    //
    // Critical: anchor on inNow (current IOProc invocation time), NOT
    // inOutputTime (future presentation time). coreaudiod rejects ZeroTimeStamp
    // host times that are far in the future ("Out of bounds host time
    // detected" in HALS_IOClock.cpp), and inOutputTime sits one IO-buffer
    // ahead — that mismatch was the source of clicks during early testing.
    Float64 realSample = 0.0;
    UInt64  realHost   = 0;
    bool    haveAnchor = false;
    if (inNow &&
        (inNow->mFlags & kAudioTimeStampSampleTimeValid) &&
        (inNow->mFlags & kAudioTimeStampHostTimeValid)) {
        realSample = inNow->mSampleTime;
        realHost   = inNow->mHostTime;
        haveAnchor = true;
    } else if (inOutputTime &&
               (inOutputTime->mFlags & kAudioTimeStampSampleTimeValid) &&
               (inOutputTime->mFlags & kAudioTimeStampHostTimeValid)) {
        // Fallback: subtract the IO-buffer offset to convert
        // future-presentation anchor → present-anchor.
        Float64 frameTicks = 1.0e9 / m->sampleRate
                              * (Float64)gTb.denom / (Float64)gTb.numer;
        UInt64 bufferTicks = (UInt64)((Float64)frames * frameTicks);
        realSample = inOutputTime->mSampleTime - (Float64)frames;
        realHost   = (inOutputTime->mHostTime > bufferTicks)
                     ? (inOutputTime->mHostTime - bufferTicks) : 0;
        haveAnchor = true;
    }
    if (haveAnchor) {

        if (!atomic_load_explicit(&m->driftPrimed, memory_order_acquire)) {
            // First capture: pick realRefSample so that virtual sample 0
            // corresponds to ioStartHost — the same anchor coreaudiod has
            // been extrapolating from during warmup. This keeps the
            // (sampleTime, hostTime) timeline continuous across the transition,
            // so coreaudiod doesn't re-anchor and glitch.
            UInt64 startHost = atomic_load_explicit(&m->ioStartHost,
                                                    memory_order_relaxed);
            UInt64 elapsedTicks = (realHost > startHost) ? (realHost - startHost) : 0;
            Float64 elapsedNs = (Float64)elapsedTicks * (Float64)gTb.numer
                                                       / (Float64)gTb.denom;
            Float64 elapsedSamples = elapsedNs * m->sampleRate / 1.0e9;
            m->realRefSample = realSample - elapsedSamples;
        }

        Float64 virtSample = realSample - m->realRefSample;
        if (virtSample < 0.0) virtSample = 0.0;
        UInt64 cycles = (UInt64)(virtSample / (Float64)kRingFrames);
        UInt64 zeroSample = cycles * kRingFrames;
        Float64 frac = virtSample - (Float64)zeroSample;
        UInt64 fracTicks = (UInt64)(frac * 1.0e9 / m->sampleRate
                                      * (Float64)gTb.denom / (Float64)gTb.numer);
        UInt64 zeroHost = (realHost > fracTicks) ? (realHost - fracTicks) : 0;

        // Seqlock write: bump to odd → store payload → bump to even.
        UInt64 cur = atomic_load_explicit(&m->driftSeq, memory_order_relaxed);
        atomic_store_explicit(&m->driftSeq, cur + 1, memory_order_relaxed);
        atomic_thread_fence(memory_order_release);
        m->driftSampleTime = zeroSample;
        m->driftHostTime   = zeroHost;
        atomic_thread_fence(memory_order_release);
        atomic_store_explicit(&m->driftSeq, cur + 2, memory_order_relaxed);

        if (!atomic_load_explicit(&m->driftPrimed, memory_order_relaxed))
            atomic_store_explicit(&m->driftPrimed, true, memory_order_release);
    }
    return noErr;
}

static OSStatus VM_StartIO(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 c) {
    (void)d; (void)c;
    int idx = object_to_mirror(id);
    if (idx < 0 || id != dev_id(idx) ||
        !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
        return kAudioHardwareBadObjectError;
    Mirror* m = &gMirrors[idx];

    pthread_mutex_lock(&m->mu);
    if (atomic_load_explicit(&m->ioRunning, memory_order_relaxed)) {
        pthread_mutex_unlock(&m->mu);
        return noErr;
    }

    ring_reset(&m->ring);
    m->currentGain = 0.0f;  // ramp up from silence on the first cycle to avoid a pop at stream start

    OSStatus err = AudioDeviceCreateIOProcID(m->realID, real_io_proc, m, &m->realProc);
    if (err != noErr || m->realProc == NULL) {
        VMLOG("StartIO: CreateIOProcID on real %{public}u failed: %{public}d",
              (unsigned)m->realID, (int)err);
        pthread_mutex_unlock(&m->mu);
        return err ? err : kAudioHardwareUnspecifiedError;
    }

    // Prime drift-correction state before AudioDeviceStart so the IOProc can
    // fire at any point afterward without seeing uninitialized fields.
    UInt64 startHost = mach_absolute_time();
    atomic_store_explicit(&m->ioStartHost, startHost, memory_order_relaxed);
    atomic_store_explicit(&m->driftSeq, 0, memory_order_relaxed);
    m->driftSampleTime = 0;
    m->driftHostTime   = startHost;
    m->realRefSample   = 0.0;
    atomic_store_explicit(&m->driftPrimed, false, memory_order_relaxed);
    atomic_fetch_add_explicit(&m->driftSeed, 1, memory_order_release);

    err = AudioDeviceStart(m->realID, m->realProc);
    if (err != noErr) {
        VMLOG("StartIO: AudioDeviceStart failed: %{public}d", (int)err);
        AudioDeviceDestroyIOProcID(m->realID, m->realProc);
        m->realProc = NULL;
        pthread_mutex_unlock(&m->mu);
        return err;
    }

    atomic_store_explicit(&m->ioRunning, true, memory_order_release);
    pthread_mutex_unlock(&m->mu);
    VMLOG("StartIO: forwarding mirror %{public}d → real %{public}u", idx, (unsigned)m->realID);
    return noErr;
}

static OSStatus VM_StopIO(AudioServerPlugInDriverRef d, AudioObjectID id, UInt32 c) {
    (void)d; (void)c;
    int idx = object_to_mirror(id);
    if (idx < 0 || id != dev_id(idx) ||
        !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
        return kAudioHardwareBadObjectError;
    Mirror* m = &gMirrors[idx];

    pthread_mutex_lock(&m->mu);
    if (!atomic_load_explicit(&m->ioRunning, memory_order_relaxed)) {
        pthread_mutex_unlock(&m->mu);
        return noErr;
    }

    atomic_store_explicit(&m->ioRunning, false, memory_order_release);
    if (m->realProc) {
        AudioDeviceStop(m->realID, m->realProc);
        AudioDeviceDestroyIOProcID(m->realID, m->realProc);
        m->realProc = NULL;
    }
    pthread_mutex_unlock(&m->mu);
    VMLOG("StopIO: torn down mirror %{public}d", idx);
    return noErr;
}

static OSStatus VM_GetZeroTimeStamp(AudioServerPlugInDriverRef d, AudioObjectID id,
                                    UInt32 c, Float64* outSampleTime,
                                    UInt64* outHostTime, UInt64* outSeed) {
    (void)d; (void)c;
    int idx = object_to_mirror(id);
    if (idx < 0 || !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
        return kAudioHardwareBadObjectError;
    Mirror* m = &gMirrors[idx];

    // Seqlock read: retry until we observe a stable (even) sequence number
    // across the entire payload load. RT-safe: bounded retry, no blocking.
    UInt64 sampleTime = 0, hostTime = 0;
    UInt64 s1, s2;
    int spins = 0;
    do {
        s1 = atomic_load_explicit(&m->driftSeq, memory_order_acquire);
        if (s1 & 1) { ++spins; continue; }  // writer mid-flight
        sampleTime = m->driftSampleTime;
        hostTime   = m->driftHostTime;
        atomic_thread_fence(memory_order_acquire);
        s2 = atomic_load_explicit(&m->driftSeq, memory_order_relaxed);
        ++spins;
    } while (s1 != s2 && spins < 64);  // 64 is wildly more than the writer ever needs

    if (outSampleTime) *outSampleTime = (Float64)sampleTime;
    if (outHostTime)   *outHostTime   = hostTime;
    if (outSeed)       *outSeed       = atomic_load_explicit(&m->driftSeed,
                                                              memory_order_acquire);
    return noErr;
}

static OSStatus VM_WillDoIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id,
                                     UInt32 c, UInt32 op, Boolean* w, Boolean* wp) {
    (void)d; (void)id; (void)c;
    Boolean want = (op == kAudioServerPlugInIOOperationWriteMix);
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
    (void)d; (void)s; (void)c; (void)ci; (void)secondaryBuffer;
    if (op != kAudioServerPlugInIOOperationWriteMix || !mainBuffer || fs == 0) return noErr;
    int idx = object_to_mirror(id);
    if (idx < 0 || !atomic_load_explicit(&gMirrors[idx].active, memory_order_acquire))
        return noErr;
    Mirror* m = &gMirrors[idx];
    if (!atomic_load_explicit(&m->ioRunning, memory_order_acquire)) return noErr;
    float scalar = atomic_load_explicit(&m->volume, memory_order_relaxed);
    bool muted = atomic_load_explicit(&m->muted, memory_order_relaxed);
    float targetGain = muted ? 0.0f : scalar_to_gain(scalar);
    float startGain  = m->currentGain;
    ring_write_scaled(&m->ring, (const float*)mainBuffer, fs, startGain, targetGain);
    m->currentGain = targetGain;
    return noErr;
}

static OSStatus VM_EndIOOperation(AudioServerPlugInDriverRef d, AudioObjectID id,
                                  UInt32 c, UInt32 op, UInt32 fs,
                                  const AudioServerPlugInIOCycleInfo* ci) {
    (void)d; (void)id; (void)c; (void)op; (void)fs; (void)ci;
    return noErr;
}

// ===========================================================================
// Enumeration (deferred)
// ===========================================================================
static bool real_is_output_with_locked_volume(AudioObjectID dev) {
    // Never mirror our own devices — they expose volume via a control object,
    // not as a direct device property, so the heuristic below would
    // misclassify them as "locked" and we'd recursively mirror ourselves.
    char uid[256] = {0};
    copy_device_uid(dev, uid, sizeof(uid));
    if (strncmp(uid, "VolMirror/", 10) == 0) return false;

    // Has output streams?
    AudioObjectPropertyAddress streams = {
        kAudioDevicePropertyStreams, kAudioObjectPropertyScopeOutput,
        kAudioObjectPropertyElementMain
    };
    UInt32 sz = 0;
    if (AudioObjectGetPropertyDataSize(dev, &streams, 0, NULL, &sz) != noErr) return false;
    if (sz == 0) return false;

    // Master output volume scalar — settable means the OS can already drive it,
    // so we leave the device alone. Missing property entirely (typical of
    // digital outputs) counts as "locked" and we mirror it.
    AudioObjectPropertyAddress vol = {
        kAudioDevicePropertyVolumeScalar, kAudioObjectPropertyScopeOutput,
        kAudioObjectPropertyElementMain
    };
    Boolean settable = false;
    if (AudioObjectIsPropertySettable(dev, &vol, &settable) == noErr && settable)
        return false;
    return true;
}

static void copy_device_name(AudioObjectID dev, char* dst, size_t dstSize) {
    AudioObjectPropertyAddress name = {
        kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    CFStringRef cf = NULL;
    UInt32 sz = sizeof(cf);
    if (AudioObjectGetPropertyData(dev, &name, 0, NULL, &sz, &cf) == noErr && cf) {
        if (!CFStringGetCString(cf, dst, (CFIndex)dstSize, kCFStringEncodingUTF8))
            snprintf(dst, dstSize, "Output %u", (unsigned)dev);
        CFRelease(cf);
    } else {
        snprintf(dst, dstSize, "Output %u", (unsigned)dev);
    }
}

// Real device's current nominal sample rate. Captured at reconcile-add time so the
// virtual mirror's rate matches the consumer's rate (otherwise the ring under-/overruns).
// If the device's rate changes after this point (rare), the user has to unplug+replug.
static Float64 device_nominal_rate(AudioObjectID dev) {
    AudioObjectPropertyAddress a = {
        kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    Float64 r = 0.0;
    UInt32 sz = sizeof(r);
    if (AudioObjectGetPropertyData(dev, &a, 0, NULL, &sz, &r) == noErr && r > 0)
        return r;
    return kFallbackSampleRate;
}

// Real device's stable UID (e.g. "AppleHDAEngineOutputDP:..."). Empty string on failure.
static void copy_device_uid(AudioObjectID dev, char* dst, size_t dstSize) {
    dst[0] = 0;
    AudioObjectPropertyAddress uid = {
        kAudioDevicePropertyDeviceUID, kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    CFStringRef cf = NULL;
    UInt32 sz = sizeof(cf);
    if (AudioObjectGetPropertyData(dev, &uid, 0, NULL, &sz, &cf) == noErr && cf) {
        CFStringGetCString(cf, dst, (CFIndex)dstSize, kCFStringEncodingUTF8);
        CFRelease(cf);
    }
}

// ===========================================================================
// Hotplug debounce
// ===========================================================================
// Debounced reconcile: bursty kAudioHardwarePropertyDevices events (HDMI plug
// fires 5-10, coreaudiod shutdown fires more) collapse into one scan after
// the burst settles. Also reduces the chance of a reconcile being mid-flight
// when coreaudiod tries to exit, which is the root cause of `make reload`
// occasionally needing `kill -9`.
static void schedule_reconcile(void) {
    if (!gReconcileTimer) return;
    dispatch_source_set_timer(gReconcileTimer,
        dispatch_time(DISPATCH_TIME_NOW, 100 * NSEC_PER_MSEC),
        DISPATCH_TIME_FOREVER, 10 * NSEC_PER_MSEC);
}

// Reconcile our mirror set with the system's current locked-output devices.
// Idempotent: safe to call at startup, after hotplug, or repeatedly.
static void reconcile_devices(void) {
    AudioObjectPropertyAddress addr = {
        kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMain
    };
    UInt32 sz = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &addr, 0, NULL, &sz) != noErr) return;
    UInt32 nDev = sz / sizeof(AudioObjectID);
    if (nDev == 0) return;
    AudioObjectID* devs = (AudioObjectID*)malloc(sz);
    if (!devs) return;
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &addr, 0, NULL, &sz, devs) != noErr) {
        free(devs); return;
    }

    // Snapshot currently-locked outputs.
    typedef struct { AudioObjectID id; char uid[224]; char name[256]; } Locked;
    Locked locked[kMaxMirrors * 2];
    int nLocked = 0;
    int cap = (int)(sizeof(locked) / sizeof(locked[0]));
    for (UInt32 i = 0; i < nDev && nLocked < cap; i++) {
        if (!real_is_output_with_locked_volume(devs[i])) continue;
        Locked* l = &locked[nLocked];
        l->id = devs[i];
        copy_device_uid(devs[i], l->uid, sizeof(l->uid));
        copy_device_name(devs[i], l->name, sizeof(l->name));
        if (l->uid[0]) nLocked++;  // require a stable UID
    }
    free(devs);

    pthread_mutex_lock(&gListMu);
    bool changed = false;

    // 1. Remove mirrors whose real device disappeared.
    for (int i = 0; i < kMaxMirrors; i++) {
        if (!atomic_load_explicit(&gMirrors[i].active, memory_order_acquire)) continue;
        bool present = false;
        for (int j = 0; j < nLocked; j++) {
            if (strcmp(gMirrors[i].realUID, locked[j].uid) == 0) {
                present = true;
                gMirrors[i].realID = locked[j].id;  // refresh runtime ID
                break;
            }
        }
        if (!present) {
            VMLOG("reconcile: removing mirror %{public}d (%{public}s)",
                  i, gMirrors[i].displayName);
            pthread_mutex_lock(&gMirrors[i].mu);
            atomic_store_explicit(&gMirrors[i].ioRunning, false, memory_order_release);
            if (gMirrors[i].realProc) {
                AudioDeviceStop(gMirrors[i].realID, gMirrors[i].realProc);
                AudioDeviceDestroyIOProcID(gMirrors[i].realID, gMirrors[i].realProc);
                gMirrors[i].realProc = NULL;
            }
            pthread_mutex_unlock(&gMirrors[i].mu);
            // Ring + mutex stay allocated for the slot's lifetime — see
            // ring_init's comment.
            atomic_store_explicit(&gMirrors[i].active, false, memory_order_release);
            changed = true;
        }
    }

    // 2. Add mirrors for newly-present locked devices.
    for (int j = 0; j < nLocked; j++) {
        bool already = false;
        for (int i = 0; i < kMaxMirrors; i++) {
            if (atomic_load_explicit(&gMirrors[i].active, memory_order_acquire) &&
                strcmp(gMirrors[i].realUID, locked[j].uid) == 0) {
                already = true; break;
            }
        }
        if (already) continue;
        int slot = -1;
        for (int i = 0; i < kMaxMirrors; i++)
            if (!atomic_load_explicit(&gMirrors[i].active, memory_order_acquire)) {
                slot = i; break;
            }
        if (slot < 0) {
            VMLOG("reconcile: no free slot for %{public}s", locked[j].name);
            continue;
        }
        Mirror* m = &gMirrors[slot];
        m->realID = locked[j].id;
        snprintf(m->realUID, sizeof(m->realUID), "%s", locked[j].uid);
        snprintf(m->baseName, sizeof(m->baseName), "%s", locked[j].name);
        snprintf(m->displayName, sizeof(m->displayName), "%s (Vol)", m->baseName);
        snprintf(m->uid, sizeof(m->uid), "VolMirror/%s", m->realUID);
        snprintf(m->modelUID, sizeof(m->modelUID), "com.joaopedro.VolMirror.model");
        m->sampleRate = device_nominal_rate(locked[j].id);
        atomic_store_explicit(&m->volume, 1.0f, memory_order_relaxed);
        atomic_store_explicit(&m->muted, false, memory_order_relaxed);
        atomic_store_explicit(&m->ioRunning, false, memory_order_relaxed);
        atomic_store_explicit(&m->ioStartHost, 0, memory_order_relaxed);
        m->realProc = NULL;
        atomic_store_explicit(&m->active, true, memory_order_release);
        VMLOG("reconcile: added mirror %{public}d (%{public}s, %{public}.0f Hz)",
              slot, m->displayName, m->sampleRate);
        changed = true;
    }
    pthread_mutex_unlock(&gListMu);

    if (changed && gHost) {
        AudioObjectPropertyAddress notify = {
            kAudioPlugInPropertyDeviceList, kAudioObjectPropertyScopeGlobal,
            kAudioObjectPropertyElementMain
        };
        gHost->PropertiesChanged(gHost, kPlugInObjectID, 1, &notify);
    }
}

// Reconcile runs at load time and again whenever the system fires a
// kAudioHardwarePropertyDevices change (see VM_Initialize). All scans go
// through gReconcileQ, so a hotplug burst can never run reconcile_devices
// concurrently with itself.

// ===========================================================================
// Lifecycle stubs
// ===========================================================================
static OSStatus VM_Initialize(AudioServerPlugInDriverRef d, AudioServerPlugInHostRef h) {
    (void)d;
    VMLOG("VM_Initialize called");
    gHost = h;
    mach_timebase_info(&gTb);
    // Eagerly init every slot's mutex + ring once, so reconcile_devices never
    // has to (re-)init in place and the RT producer never sees a freed buffer.
    for (int i = 0; i < kMaxMirrors; i++) {
        pthread_mutex_init(&gMirrors[i].mu, NULL);
        ring_init(&gMirrors[i].ring, kRingCapacity);
        atomic_init(&gMirrors[i].active, false);
        atomic_init(&gMirrors[i].volume, 1.0f);
        atomic_init(&gMirrors[i].muted, false);
        atomic_init(&gMirrors[i].ioRunning, false);
        atomic_init(&gMirrors[i].ioStartHost, 0);
        atomic_init(&gMirrors[i].driftSeq, 0);
        atomic_init(&gMirrors[i].driftPrimed, false);
        atomic_init(&gMirrors[i].driftSeed, 1);
        gMirrors[i].driftSampleTime = 0;
        gMirrors[i].driftHostTime = 0;
        gMirrors[i].realRefSample = 0.0;
        gMirrors[i].sampleRate = kFallbackSampleRate;
        gMirrors[i].currentGain = 0.0f;
    }
    // Serial queue: the initial scan and every hotplug-driven scan run one
    // at a time, so reconcile_devices never overlaps with itself even under
    // a burst of plug events.
    gReconcileQ = dispatch_queue_create("com.joaopedro.VolMirror.reconcile",
                                         DISPATCH_QUEUE_SERIAL);
    // No XPC calls inline (they can hang during plugin init). Initial scan,
    // listener registration, and debounce-timer setup all run on gReconcileQ
    // after VM_Initialize has returned.
    dispatch_async(gReconcileQ, ^{
        reconcile_devices();
        // Debounce timer for hotplug-driven reconciles. One-shot per arming;
        // schedule_reconcile() re-arms it.
        gReconcileTimer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER,
                                                  0, 0, gReconcileQ);
        dispatch_source_set_event_handler(gReconcileTimer, ^{ reconcile_devices(); });
        dispatch_resume(gReconcileTimer);

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
        if (err != noErr)
            VMLOG("hotplug: AddPropertyListenerBlock failed: %{public}d", (int)err);
        else
            VMLOG("hotplug: listening for kAudioHardwarePropertyDevices");
    });
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
    VMLOG("VolMirror_Factory called");
    if (typeUUID == NULL) { VMLOG("Factory: typeUUID null"); return NULL; }
    if (!CFEqual(typeUUID, kAudioServerPlugInTypeUUID)) {
        VMLOG("Factory: typeUUID mismatch");
        return NULL;
    }
    VMLOG("Factory: returning interface ptr");
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
