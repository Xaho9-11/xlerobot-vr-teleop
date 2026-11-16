# Control the Quest 3 from this PC over ADB (2026-11-13; user: no headset on for routine operations).
# Usage:  .\quest.ps1 <command> [arg]
#   wifi            once after every Quest reboot, with the USB cable plugged in: switch ADB to Wi-Fi and connect
#   connect         reconnect ADB over Wi-Fi (Quest address in $QuestIp)
#   status          ADB link, battery, Wi-Fi address, whether the teleop app runs, its saved Robot IP
#   ip lab|home|x   set the app's saved Robot IP (lab = Cudy 192.168.50.113, home = Xiaomi 192.168.31.113);
#                   stops the app first, since it rewrites its settings on exit
#   start / stop    start or stop the teleop app
#   awake / normal  keep the headset awake while not worn (proximity sensor treated as "worn") / back to normal
#   log             follow the app's Unity log (Ctrl+C to stop)
# The app is a debuggable build, so its settings file can be edited with run-as (no app change needed).
param([Parameter(Mandatory = $true)][string]$Cmd, [string]$Arg)

# adb: $env:QUEST_ADB, else the copy that ships with the Unity Android module on this PC, else adb on the PATH
$Adb = $env:QUEST_ADB
if (-not $Adb) { $Adb = 'X:\Unity_hub\Editor\6000.3.9f1\Editor\Data\PlaybackEngines\AndroidPlayer\SDK\platform-tools\adb.exe' }
if (-not (Test-Path $Adb)) { $Adb = 'adb' }
# Quest address on the lab network ($env:QUEST_IP overrides; bind it to a fixed address in the router)
$QuestIp = if ($env:QUEST_IP) { $env:QUEST_IP } else { '192.168.50.216' }
$Serial = "$QuestIp`:5555"
$Pkg = 'com.DefaultCompany.ReachyMiniTeleop'
$Act = "$Pkg/com.unity3d.player.UnityPlayerGameActivity"
$Prefs = "shared_prefs/$Pkg.v2.playerprefs.xml"
$Robot = @{ lab = '192.168.50.113'; home = '192.168.31.113' }

function Q { & $Adb -s $Serial shell @args }

switch ($Cmd) {
  'wifi' {
    & $Adb -d tcpip 5555; Start-Sleep 3
    $ip = (& $Adb -d shell "ip -4 addr show wlan0" | Select-String 'inet (\d+\.\d+\.\d+\.\d+)').Matches.Groups[1].Value
    if ($ip) { & $Adb connect "$ip`:5555"; if ($ip -ne $QuestIp) { "Quest is at $ip - update `$QuestIp in this script (or bind it in the router)" } }
  }
  'connect' { & $Adb connect $Serial }
  'status' {
    & $Adb devices
    "battery: " + ((Q dumpsys battery) -match 'level' | Select-Object -First 1).Trim()
    "wifi:    " + ((Q ip -4 addr show wlan0) -match 'inet' | Select-Object -First 1).Trim()
    "app pid: " + (Q pidof $Pkg)
    "app ip:  " + (Q run-as $Pkg grep LastRobotIp $Prefs)
  }
  'ip' {
    $to = if ($Robot.ContainsKey($Arg)) { $Robot[$Arg] } else { $Arg }
    if ($to -notmatch '^\d+\.\d+\.\d+\.\d+$') { throw "usage: quest.ps1 ip lab|home|<address>" }
    Q am force-stop $Pkg
    # one quoted string for the Quest shell (split arguments let "|" act as a pipe there); the file has a single
    # <string> entry, the Robot IP, so the expression needs no double quotes
    Q "run-as $Pkg sed -i 's|>[0-9.]*</string>|>$to</string>|' $Prefs"
    Q "run-as $Pkg grep LastRobotIp $Prefs"
  }
  'start'  { Q am start -n $Act }
  'stop'   { Q am force-stop $Pkg }
  'awake'  { Q am broadcast -a com.oculus.vrpowermanager.prox_close }
  'normal' { Q am broadcast -a com.oculus.vrpowermanager.automation_disable }
  'log'    { & $Adb -s $Serial logcat -s Unity }
  default  { "unknown command: $Cmd" }
}
