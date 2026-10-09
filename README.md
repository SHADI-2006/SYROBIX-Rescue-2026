# SYROBIX Rescue 2026 — Robot Firmware

![PlatformIO](https://img.shields.io/badge/PlatformIO-Arduino-orange) ![MCU](https://img.shields.io/badge/MCU-ESP32--S3-blue) ![Build](https://img.shields.io/badge/build-passing-brightgreen) ![Status](https://img.shields.io/badge/robot%20tested-not%20yet-yellow)

Firmware for an autonomous 4WD skid-steer **rescue robot** built for the **SYROBIX Rescue 2026**
university competition. The robot follows a black line, handles intersections and dead ends from
green markers, bypasses obstacles, defuses mines on red tiles, rescues a victim from the green safe
tile and delivers it to the exit tile.

> **بالعربي:** برمجيات روبوت إنقاذ ذاتي القيادة (ESP32-S3) لمسابقة SYROBIX Rescue 2026: يتبع الخط،
> يقرأ علامات التقاطعات الخضراء، يتجاوز العوائق، يفكّك الألغام على البلاطات الحمراء، وينقذ الضحية
> إلى بلاطة الخروج.

## Status
- Builds cleanly with the real PlatformIO toolchain: **0 warnings**, Flash ≈ 10 %, RAM ≈ 6 %.
- Host-side simulations (no hardware) pass: addressing of the ToF sensors and the whole mission
  state machine (intersections, mines, obstacles, victim, retries, PCA9685 brown-out recovery).
- **Not yet run on the real robot.** Every tuning constant in `src/Config.h` is a starting value
  and the physical measurements listed in `CLAUDE.md` still have to be taken.

## Hardware
| Part | Details |
|---|---|
| MCU | ESP32-S3 DevKitC |
| Drive | 4× JGA25-370 motors on 2× TB6612FNG, one quadrature encoder per side |
| Line sensing | 8× TCR5000 on ADC1 (6 front row + 2 rear) |
| Distance | 3× VL53L1X ToF (front / left / right), re-addressed at boot via XSHUT |
| Orientation | MPU6500 IMU (gyro-Z yaw, pitch for ramp detection) |
| Colour | 2× TCS3200 |
| Servos / LED / buzzer | PCA9685 on the shared I²C bus (arm MG996R, gripper MG90S) |

## Software architecture
Five FreeRTOS tasks split over the two cores (control and line sensing at 200 Hz on core 1;
sensors, mission and telemetry on core 0). Only the control task writes the motors, and core 1
never touches I²C. All constants live in `src/Config.h` with compile-time `static_assert` checks.
See [`CLAUDE.md`](CLAUDE.md) for the full task map and the rules of the road.

```
src/          firmware (Config, LineArray, LidarBank, SensorsSystem, RobotDrivetrain, TaskHandler, main)
host_tests/   hardware-free simulations of the sensors and the mission logic
docs/         CAD and printed-chassis photos
```

## Build and flash
```bash
pip install platformio
pio run                 # build
pio run -t upload       # flash (use the board's UART/COM USB port)
pio device monitor      # serial telemetry, 115200 baud
```
`platformio.ini` pins `espressif32 @ ^6.9.0` (Arduino-ESP32 core 2.0.x) on purpose.

## Host tests
Needs `g++` with C++17 (Linux, WSL or MSYS2):
```bash
cd host_tests && sh run.sh
```

## Hardware bench tools
- [`bench/`](bench/README.md) — standalone hardware test firmware + single-file web UI (Web Serial).
  Checks every part on its own (IMU, ToF, line array, colour, motors/encoders, servos, LED,
  buzzer, PCA9685) and produces measured values ready to paste into `src/Config.h`.
  Builds independently of the match firmware: `pio run -d bench`.
- [`bench_wifi/`](bench_wifi/README.md) — the same tool and the **same UI** over Wi-Fi, plus a
  **practice run** mode that boots the real match firmware with remote START / STOP and live
  mission state. **Practice only:** the rulebook forbids wireless links during a scored round.

## Competition rules
The official rulebook is published by the SYROBIX organisers and is **not** included in this
repository; the mission constants in `src/Config.h` were derived from it.

## License
No license has been chosen yet; all rights reserved by the authors until one is added.
