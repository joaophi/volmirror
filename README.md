# VolMirror

A macOS CoreAudio plugin + companion agent that adds a software-volume mirror for
every audio output whose hardware volume is locked (typically HDMI, DisplayPort,
or optical).

No menu bar app, no preferences window. Mirrors appear in the standard
**System Settings → Sound** output picker, with the same icon as the real device.

```
Sound Outputs:
  MacBook Speakers
  LG ULTRAWIDE
  LG ULTRAWIDE (Vol)      ← mirror, OS slider drives software gain
  Samsung TV
  Samsung TV (Vol)        ← mirror, OS slider drives software gain
```

Picking `(Vol)` routes audio through VolMirror, which applies a quadratic-tapered
gain (driven by the OS volume slider / F11/F12 / mute key) and forwards samples
to the paired real device.

## Architecture

Two pieces:

```
┌──────────────┐   apps write audio    ┌────────────────┐
│  /Apps/...   │ ─────────────────────▶│  VolMirror      │   driver runs in
│  (Spotify,   │                        │  .driver        │   coreaudiod's
│   Music, …)  │                        │  (HAL plugin)   │   process
└──────────────┘                        └────────┬────────┘
                                                 │ ring buffer
                                                 │ (gain applied)
                                                 ▼
┌──────────────┐                        ┌─────────────────┐
│  Real device │ ◀──────────────────────│ VolMirrorAgent  │   user-session
│  (HDMI / DP) │   agent writes samples │ (LaunchAgent)   │   process
└──────────────┘                        └────────┬────────┘
                                                 │ assigns slots,
                                                 │ handles hotplug,
                                                 ▼
                                        ┌─────────────────┐
                                        │   real outputs  │   read via the
                                        │   (CoreAudio)   │   public HAL API
                                        └─────────────────┘
```

**`Source/Plugin.c` — the driver.** Pre-allocates 8 hidden duplex slots. Owns the
volume control, mute control, and an SPSC ring buffer per slot. No client-HAL
calls, no real-device enumeration, no IOProcs against other devices. Apps write
audio; the driver applies volume gain (with click-avoiding ramp); the audio sits
in the ring waiting for the agent to drain it.

The driver exposes custom properties the agent uses to assign each slot:

| Selector | Type    | Purpose                                      |
|----------|---------|----------------------------------------------|
| `'flwt'` | CFString| Real device UID. `""` unassigns (slot hides).|
| `'fldn'` | CFString| Display name shown in the UI.                |
| `'flic'` | CFString| File path to icon `.icns`.                   |
| `'flmf'` | CFString| Manufacturer string (overrides "VolMirror"). |
| `'flsr'` | CFString| Nominal sample rate (ASCII Float64).         |
| `'fltt'` | CFString| Transport type (4-char fourcc, e.g. "hdmi"). |

**`Source/Agent.c` — the routing brain.** A user-session LaunchAgent. Uses the
standard CoreAudio client API to enumerate real devices, filter to those with
locked hardware volume, and assign each to a free virtual slot. For every active
mirror it runs an IO bridge:

- Input IOProc on the virtual slot → drains the driver's ring → its own ring.
- Output IOProc on the real device → consumes from its ring → real hardware.

It also listens for `kAudioHardwarePropertyDevices` (hotplug) on a 100 ms
debounce and reconciles assignments when displays come and go.

### Why split it?

Earlier versions ran everything inside the driver and called the client HAL API
to enumerate and follow real devices — exactly what `AudioServerPlugIn.h`
explicitly tells plugins not to do. It worked, but every iteration on routing
required killing `coreaudiod`, which leaves Control Center and audio apps
briefly wedged.

The split lets you reload routing logic without bouncing `coreaudiod`:

```sh
sudo make reload-agent      # restart routing logic, no coreaudiod kill
sudo make reload-driver     # only when changing the driver itself
sudo make reload            # both
```

Day-to-day iteration touches the agent. The driver is small and rarely changes.

## Build

Requires only macOS Command Line Tools (`xcode-select --install`).

```sh
make
```

Builds the driver bundle and the agent binary. Both are ad-hoc code-signed.

## Install

```sh
sudo make install
```

This:

- Copies `VolMirror.driver` → `/Library/Audio/Plug-Ins/HAL/`.
- Copies `VolMirrorAgent` → `/usr/local/libexec/`.
- Copies the launchd plist → `/Library/LaunchAgents/`.
- Strips the quarantine xattr and warms Gatekeeper on the agent binary so the
  first-launch malware scan happens during install instead of stalling
  launchd at next login.
- Restarts `coreaudiod` (loads the driver) and bootstraps the agent.

After install, the `(Vol)` entries appear in System Settings → Sound within a
second.

### First-time microphone prompt

The agent reads from the virtual device's input stream to drain the ring. macOS
treats *any* input-stream IOProc as recording, so you'll see a microphone
permission prompt the first time the agent runs. Click Allow once. The orange
"recording" indicator in the menu bar will be on while the agent is bridging
audio — same trade-off every loopback driver makes.

## Uninstall

```sh
sudo make uninstall
```

Removes the driver, agent binary, and launchd plist; bootouts the agent and
restarts `coreaudiod`.

## Constraints

- **Stereo Float32 LinearPCM only.** No multichannel, no bitstream/passthrough.
- **No sample-rate conversion.** The agent matches the virtual slot's sample
  rate to the real device's nominal rate at assignment time. If the real
  device changes rate after assignment, you'll need to unplug/replug.
- **Personal-use bar.** Ad-hoc signed; runs on this Mac only. Sharing with
  other Macs would require a paid Apple Developer ID and notarization.
- **First boot after install is slow.** Gatekeeper does its malware scan on
  the new agent binary on first launch; subsequent boots are fast until the
  next install.
