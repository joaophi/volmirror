# VolMirror

A macOS CoreAudio HAL plugin that adds a volume-controllable mirror entry for every
audio output that has its hardware volume locked (typically HDMI / DisplayPort / optical).

No menu bar app, no preferences window, no UI at all — the mirrors only show up in the
standard System Settings → Sound output picker.

## How it looks

```
Sound Outputs:
  MacBook Speakers
  LG ULTRAFINE
  LG ULTRAFINE (Vol)      ← mirror, software volume works
  Samsung TV
  Samsung TV (Vol)        ← mirror, software volume works
```

Picking a `(Vol)` device routes audio through the plugin, which applies a software
gain (driven by the standard macOS volume slider / F11 / F12) and forwards samples
to the paired real device.

## Architecture

A single `.driver` bundle loaded by `coreaudiod`. On load:

1. Enumerate all output `AudioDevice`s via the HAL.
2. For each one where `kAudioDevicePropertyVolumeScalar` is **not settable**, publish
   a paired virtual device named `"<RealName> (Vol)"`.
3. Each virtual device:
   - Reports `kAudioDevicePropertyVolumeScalar` as settable (so the OS slider writes to it).
   - Has its own IOProc that scales incoming samples by the volume scalar and pushes
     them into a small lock-free SPSC ring buffer.
   - Registers a second IOProc on the **paired real device** that drains the ring and
     writes to the real output buffer.
4. Listens for `kAudioHardwarePropertyDevices` changes on a serial queue and
   re-runs the reconciler to add/remove mirrors as monitors are plugged or
   unplugged.

Stereo Float32 PCM only. No multichannel, no bitstream — if you need surround, use
the original output.

## Build

Only requirement: macOS Command Line Tools (`xcode-select --install`). No Xcode app
needed.

```sh
make
```

Builds and ad-hoc signs `build/VolMirror.driver`.

## Install

```sh
sudo make install
```

This copies the bundle to `/Library/Audio/Plug-Ins/HAL/` and restarts `coreaudiod`.

If macOS prompts about a system extension, open **System Settings → Privacy &
Security**, scroll down, click **Allow**, then:

```sh
sudo make reload
```

The `(Vol)` entries should now appear in System Settings → Sound.

To remove:

```sh
sudo make uninstall
```

### Why `codesign -s -`?

`coreaudiod` refuses to load unsigned code. The `-` identity is **ad-hoc signing** —
free, no Apple Developer account needed, but only valid on this machine. If you want
to share the plugin with other Macs, you'll need a paid Developer ID and notarization.

## Roadmap

- [x] Project scaffold (bundle layout, Info.plist, plugin entry point)
- [x] Enumerate real outputs and publish a `(Vol)` mirror per locked one
- [x] Forward samples through ring buffer with software gain
- [x] Build + ad-hoc sign + install script
- [x] Hotplug support (`kAudioHardwarePropertyDevices` listener)
