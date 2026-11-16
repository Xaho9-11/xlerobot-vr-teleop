# Quest app and headset control

The headset runs [HRI-EU/reachy-vr-quest](https://github.com/HRI-EU/reachy-vr-quest) (Unity, MIT licence, Honda
Research Institute Europe), a teleoperation client for the Reachy Mini. The robot side of this repository speaks
its protocol (pose WebSocket on `:8000`, WebRTC signalling on `:8443`), so the app needs only small changes.

## The patch

`reachy-vr-quest-fmp.patch` applies on top of upstream commit `242120e`:

```bash
git clone https://github.com/HRI-EU/reachy-vr-quest.git
cd reachy-vr-quest && git checkout 242120e
git apply ../reachy-vr-quest-fmp.patch
```

What it changes:

| File | Change |
|---|---|
| `Reachy/FmpXrInput.cs` (new) | Sends both controllers' poses, grip, trigger, buttons and thumbstick clicks with the headset pose, for arm teleoperation (upstream sends the head only). |
| `Reachy/ReachyDaemonPayloadDtos.cs`, `ReachyDaemonTargetAdapter.cs`, `ReachyHeadCommandPublisher.cs` | Carry the controller data in the pose messages. |
| `Transport/ReachyDaemonTargetWebSocketClient.cs` | TCP_NODELAY (small pose messages were batched by Nagle and arrived in pairs, ~100 ms gaps) and an open-state loop check instead of a per-read ping (sending was capped near 9 Hz). |
| `Reachy/FmpAutoConnect.cs` (new) | Connects pose and video on start with the saved robot address, reconnects every 3 s after a drop, respects a manual switch-off; skips the Reachy Mini "wake" REST call (plain HTTP, not used by this robot). Menu hiding once connected (left menu button toggles it) is written but not yet built or tested on the headset. |
| `FmpTools/Editor/FmpBatch.cs` (new) | Batch-mode NuGet restore and Android build. |
| Scene and settings | Video surface and menu placement, build settings for Quest 3. |

Build: `Unity -batchmode -nographics -projectPath <project> -buildTarget Android -executeMethod FmpTools.FmpBatch.BuildAndroid -quit`.
The Quest 3 needs the IL2CPP backend (ARM64). On Windows with Smart App Control enforced, the unsigned
`Unity.IL2CPP.dll` is blocked and the build fails with `0x800711C7`.

## quest.ps1

Controls the headset from the PC over ADB (developer mode, Wi-Fi ADB enabled once over USB with `quest.ps1 wifi`):

```
quest.ps1 status            ADB link, battery, Wi-Fi address, app running, saved robot address
quest.ps1 ip lab|home|x.x.x.x   set the app's saved robot address (the build is debuggable; edited via run-as)
quest.ps1 start | stop      start / stop the app (it connects by itself)
quest.ps1 awake | normal    keep the headset awake while not worn / back to normal
quest.ps1 log               follow the app log
```

Set `QUEST_ADB` and `QUEST_IP` to override the adb path and the headset address.
