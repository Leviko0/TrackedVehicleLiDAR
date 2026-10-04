# TrackedVehicleLiDAR

Tracked vehicle with a 360° LiDAR, controlled from your phone through a web page
hosted on an **ESP32-S3-N16R8**.

- Hardware docs: [Power path](docs/PowerPath.md) · [Signal path](docs/SignalPath.md) · [Pin table](docs/PinTable.md)

## Web remote control

| Left side | Right side |
| --- | --- |
| Virtual joystick: up/down = forward/backward, left/right = turn | ▲ forward · ▼ backward · ↺ spin counter-clockwise · ↻ spin clockwise |

The header shows the connection state, live output of both tracks, a speed
limit slider and a **STOP** button. Hold the phone in landscape.

Safety built in:

- **Failsafe** – motors stop if no command arrives for 500 ms (Wi-Fi lost,
  browser closed, phone locked).
- **Ramping** – track speed changes gradually to protect the gearboxes and the
  12 V supply.
- Motors stop when a client disconnects, the page is hidden, or STOP is pressed.

## Getting started

1. Install [VS Code](https://code.visualstudio.com/) with the
   [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
   and open this folder.
2. *(Optional)* To join your home Wi-Fi, copy `include/Secrets.example.h` to
   `include/Secrets.h` and enter SSID and password. `Secrets.h` is git-ignored.
3. Connect the ESP32-S3 and click **Upload** (or run `pio run -t upload`).
4. Open the **Serial Monitor** (115200 baud). It prints the address, e.g.
   `open http://192.168.4.1/ on your phone`.
5. Connect your phone:
   - **Access point mode** (default, or when home Wi-Fi fails): join the Wi-Fi
     `TrackedVehicle` (password `drive1234`) and open `http://192.168.4.1`.
   - **Station mode**: phone on the same network, open the printed IP or
     `http://tank.local`.

Wi-Fi names, pins, speeds, ramp, timeout etc. are all in
[`include/Config.h`](include/Config.h).

### First drive checklist

- Jack the vehicle up so the tracks are off the ground.
- Press ▲. If a track spins backwards, set `LEFT_INVERTED`/`RIGHT_INVERTED` in
  `Config.h` (no rewiring needed).
- If the tracks only start moving at higher speeds, raise `motor::MIN_DUTY`;
  if they creep or jerk at low speed, lower it.

## Project structure

```
include/
  Config.h               all pins and tunables
  Secrets.example.h      template for Wi-Fi credentials
lib/DriveCore/           hardware independent logic (unit tested on the PC)
  DriveMath.*            arcade mixing, deadband, ramping, duty mapping
  CommandParser.*        text protocol between web page and vehicle
src/
  main.cpp               wires the modules together
  motor/Motor.*          one L298N channel (direction pins + PWM)
  drive/DriveController.*  command -> smooth track speeds, failsafe
  net/WifiManager.*      station mode with access point fallback, mDNS
  web/WebInterface.*     HTTP server + WebSocket, telemetry
  web/WebPage.h          the phone UI (HTML/CSS/JS)
test/test_drive_core/    Unity tests for DriveCore
```

Data flow: `phone --WebSocket--> WebInterface --Command--> DriveController --> Motor x2`

### WebSocket protocol (`/ws`)

| Message | Meaning |
| --- | --- |
| `D <throttle> <turn>` | drive; both in [-1, 1], turn > 0 = clockwise |
| `S` | stop immediately |
| `L <limit>` | speed limit in [0, 1] |
| `P` | keep-alive |

The page sends the current command every 50 ms; the vehicle answers with
telemetry `{"l":..,"r":..,"lim":..,"fs":..}` every 200 ms.

## Development

```bash
pio run -e esp32s3            # build firmware
pio run -e esp32s3 -t upload  # flash
pio device monitor            # serial output
pio test -e native            # run unit tests on the PC
```

GitHub Actions runs the unit tests and builds the firmware on every push.

## Hardware notes

- The L298N drops about 2–3 V, so the motors see roughly 9–10 V from the 12 V rail.
- ESP32-S3 GPIOs are **3.3 V only**. If the LiDAR's TX line idles at 5 V, put a
  level shifter or voltage divider between LiDAR TX and GPIO 3.
- The L298N logic inputs accept the ESP32's 3.3 V levels.
- GPIO 35–37 are used by the octal PSRAM on the N16R8 and must stay free (they are).
