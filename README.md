# TrackedVehicleLiDAR

Tracked vehicle with a 360° **COIN-D6** LiDAR, controlled from your phone through a
web page hosted on an **ESP32-S3-N16R8**.

- Hardware docs: [Power path](docs/PowerPath.md) · [Signal path](docs/SignalPath.md) · [Pin table](docs/PinTable.md)

## Web remote control

Hold the phone upright (portrait):

| Area | Content |
| --- | --- |
| Header | connection state, live output of both tracks, speed limit slider, **Guard** toggle, **STOP** |
| Top | live LiDAR point map: vehicle in the centre, front = up, rings every ¼ of the range, **+ / −** to zoom, scan rate and free distance ahead ▲ / behind ▼ |
| Bottom | virtual joystick: up/down = forward/backward, left/right = turn (full left/right spins on the spot) |

Points in the driving corridor are drawn orange in the slow-down zone and red
in the stop zone.

Safety built in:

- **Collision guard** (LiDAR) – in the driving direction the vehicle slows down
  from 60 cm and stops 15 cm before an obstacle. Turning on the spot is always
  allowed so you can steer away. Toggle it with the **Guard** button; while the
  LiDAR is offline the guard cannot see anything and the map says so.
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

### LiDAR check (do this before relying on the guard)

The COIN-D6 is mounted **backwards**, which `lidar::MOUNT_YAW_DEG = 180` in
`Config.h` compensates for. Verify it on the map:

1. Hold a box **in front** of the vehicle: it must appear **above** the vehicle.
   If it shows up below, set `MOUNT_YAW_DEG` to `0`.
2. Hold it on the vehicle's **right**: it must appear on the **right**. If it
   appears on the left, set `ANGLES_CLOCKWISE` to `false`.
3. If parts of the vehicle (or the LiDAR mount) show up as points around the
   vehicle, increase `FOOTPRINT_MARGIN_MM`.

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
lib/LidarCore/           hardware independent LiDAR logic (unit tested on the PC)
  CoinD6Decoder.*        COIN-D6 packet decoder (header, checksum, angles)
  PolarScan.*            revolution -> 360 x 1° scan in the vehicle frame
  Proximity.*            free distance in the driving corridor, speed factor
src/
  main.cpp               wires the modules together
  motor/Motor.*          one L298N channel (direction pins + PWM)
  drive/DriveController.*  command -> smooth track speeds, failsafe, guard limits
  lidar/LidarSensor.*    UART, start command, scan rate, online detection
  safety/CollisionGuard.*  scan -> throttle limits ahead / behind
  net/WifiManager.*      station mode with access point fallback, mDNS
  web/WebInterface.*     HTTP server + WebSocket, telemetry, scans
  web/WebPage.h          the phone UI (HTML/CSS/JS)
test/                    Unity tests for DriveCore and LidarCore
```

Data flow:

```
phone --WebSocket--> WebInterface --Command--> DriveController --> Motor x2
LiDAR --UART--> LidarSensor --scan--> CollisionGuard --limits--> DriveController
                                 \--> WebInterface --> phone (point map)
```

### WebSocket protocol (`/ws`)

| Message | Meaning |
| --- | --- |
| `D <throttle> <turn>` | drive; both in [-1, 1], turn > 0 = clockwise |
| `S` | stop immediately |
| `L <limit>` | speed limit in [0, 1] |
| `G <0\|1>` | collision guard off / on |
| `P` | keep-alive |

The page sends the current command every 50 ms. The vehicle sends

- on connect: `{"cfg":{"w":..,"len":..,"stop":..,"slow":..,"margin":..,"guard":..}}`
- every 200 ms: `{"l","r","lim","fs","li","hz","gd","cf","cr"}` (track outputs,
  speed limit, failsafe, LiDAR online, scan rate, guard on, free distance
  front / rear in mm)
- each LiDAR scan (max ~6/s): binary `'S', 0` + 360 × uint16 LE distances in mm,
  index = degrees clockwise from the vehicle front, 0 = no return.

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
- The COIN-D6 talks at 230400 baud. The ESP32 sends it the start command
  (`AA 55 F0 0F`) at boot and again every 2 s while no data arrives, so the
  GPIO 1 → LiDAR RX wire is needed.
- The L298N logic inputs accept the ESP32's 3.3 V levels.
- GPIO 35–37 are used by the octal PSRAM on the N16R8 and must stay free (they are).
