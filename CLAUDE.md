# SYROBIX 2026 Rescue Robot — project brief for Claude Code

ESP32-S3 firmware (PlatformIO + Arduino framework) for an autonomous 4WD skid-steer
rescue robot. University competition: SYROBIX Rescue 2026. The rulebook is in
`docs/SYROBIX-Rescue-Competition-Rules.pdf`. Photos of the CAD and the printed chassis
plates are in `docs/`.

## How to talk to the user
- The user is an Arabic speaker (Syrian/Levantine dialect). Explain in Arabic, short and
  clear. Keep code, identifiers and code comments in English.
- The user is a student building this for a competition. Explain WHY, not just what.
  Before any fix, say in one sentence what was wrong.
- The user's machine is **Windows** (project path looks like
  `C:\Users\<name>\Documents\PlatformIO\Projects\SYROBIX_Rescue_2026`). Use PowerShell/cmd
  syntax, not bash, for anything you tell them to run.

## STATUS — read this first (honest state of the code)
- The code has **never been built with the real PlatformIO toolchain**. It was checked only
  with host-side `g++ -fsyntax-only` against stub headers (plus the real Adafruit PCA9685
  and Pololu VL53L1X headers) and a host simulation of the mission logic
  (`host_tests/`). A clean host check does NOT guarantee `pio run` passes.
- It has **never run on the robot**. Every tuning constant is a starting value.
- A previous compile log from the user showed ~130 "not a member of Hw/Tune/Mission" errors
  in `TaskHandler.cpp`. Cause: the user had a MIX of old and new files in `src/`. The
  files in this package are one consistent set. Make sure `src/` contains exactly the
  files listed below and no stray copies such as `Config (1).h` or `TaskHandler (1).cpp`.

## FIRST TASK: get a clean real build
1. Check the toolchain: `pio --version` (PlatformIO Core). If missing, tell the user to
   install the PlatformIO extension in VS Code, or `pip install platformio`.
2. In the project root (where `platformio.ini` is): `pio run -t clean` then `pio run`.
   The first build downloads the espressif32 platform and libraries; this is normal.
3. Fix every compiler/linker error you find. Keep fixes minimal and local. Do not rewrite
   whole files.
4. Report to the user: which errors you saw, the root cause of each, what you changed.
5. Then run the host tests (see below) to make sure nothing regressed.

Likely real-build risks (not yet seen on the real toolchain — check these first):
- Library version resolution: `pololu/VL53L1X @ ^1.3.1`,
  `adafruit/Adafruit PWM Servo Driver Library @ ^3.0.2`, `adafruit/Adafruit BusIO`.
  If a version does not resolve, run `pio pkg search` and pin one that exists.
- Arduino-ESP32 core version: `espressif32 @ ^6.9.0` gives core 2.0.x. The motor LEDC code
  is version-gated for core 3.x as well (`ESP_ARDUINO_VERSION_MAJOR`), but 2.0.x is what
  it has been written against. Do not move to core 3.x without telling the user.
- `LidarBank.cpp` uses the Pololu `VL53L1X` class (`setBus`, `setTimeout`, `init`,
  `setAddress`, `setDistanceMode`, `setMeasurementTimingBudget` in MICROSECONDS,
  `startContinuous(period_ms)`, `dataReady`, `read(false)`, `ranging_data.range_status`,
  `last_status`). Confirm each against the installed library headers.
- `Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(Hw::PCA9685_ADDR);` is a global in
  `TaskHandler.cpp`. Adafruit's `begin()` calls `Wire.begin()` with no pins; this is only
  safe because `SensorsSystem::begin()` already started the bus on GPIO9/10 (see
  `main.cpp` boot order). Do not reorder boot.
- Stack sizes in `Config.h` `Rtos::` may need raising if a task overflows. Look for
  stack-overflow resets in the serial log.

## Hardware (what the code assumes)
- MCU: ESP32-S3 DevKitC. I2C on GPIO9 (SDA) / GPIO10 (SCL), 400 kHz.
- Motors: 4x JGA25-370 on 2x TB6612FNG. Left/right PWM+direction lines are shared per
  side (front and rear motor of a side are electrically locked). Native LEDC at 20 kHz.
  `MOTOR_STBY` (GPIO17) LOW = drivers off.
- Encoders: one per side (quadrature, interrupts).
- Line sensors: 8x TCR5000 on ADC1 (GPIO1–8). Indices 0–5 = FRONT row (position
  centroid), 6–7 = REAR pair.
- ToF: 3x VL53L1X (front/left/right) re-addressed at boot with XSHUT pins GPIO40/41/42,
  target addresses 0x30/0x31/0x32.
- IMU: MPU6500 (0x68). Gyro Z gives yaw.
- Colour: 2x TCS3200, shared S2/S3 (GPIO45/46), OUT on GPIO47/48. S0/S1 hard-strapped.
- **PCA9685 @ 0x40** (same I2C bus): ch0 arm servo (MG996R), ch1 gripper servo (MG90S),
  ch2 status LED, ch3 ACTIVE buzzer. See `namespace PcaChannels` in `Config.h`.
- Start button: GPIO0 (strapping pin, active LOW).
- Free GPIOs: 19, 20 (native USB), 43, 44 (UART0). Intentionally unused.
- Hardware cautions the firmware cannot fix:
  - The PCA9685 outputs have a 220 ohm series resistor and give ~10 mA at 3.3 V. The
    active buzzer MUST be driven through a transistor (NPN/AO3400), not directly.
  - Servo power goes to the PCA9685 **V+** terminal from a separate 5–6 V UBEC rated for
    ~2.5 A with a 470–1000 uF capacitor at the terminal and a common ground. This is what
    prevents a repeat of the servo-spike burn-out that damaged the previous board.
  - Add a hardware kill switch on the battery (rulebook asks for an emergency stop).

## Software architecture
- `src/Config.h` — single source of truth. Namespaces: `Pins`, `PcaChannels`, `Hw`, `Tune`,
  `Tof`, `Mission`, `Rtos`. Every magic number lives here. `static_assert`s catch address
  collisions and bad constants at compile time. **If you add a constant, add it here, in
  the right namespace.**
- `src/main.cpp` — boot order: drivetrain -> sensors (starts I2C) -> `TaskHandler::begin()`
  (starts PCA9685, then tasks). Order is load-bearing.
- `src/LineArray.h/.cpp` — IR array, calibration, position (-1..+1), line-lost.
- `src/LidarBank.h/.cpp` — VL53L1X bank; XSHUT re-addressing; non-blocking reads;
  distance mode (SHORT) and timing budget in ms (converted to us in ONE place).
- `src/SensorsSystem.h/.cpp` — `Imu`, `ColorVision`, `SensorSnapshot`, mutex-protected
  publisher. Calibration is done via *posted requests* executed by the sensor task.
- `src/RobotDrivetrain.h/.cpp` — motors, encoders, line PID, odometry. Only the control
  task may call methods that write motors.
- `src/TaskHandler.h/.cpp` — `PcaBus`, `Manipulator`, `Indicator`, the 5 FreeRTOS tasks and
  the mission state machine.

### Task map
| Core | Task | Rate | Job |
|---|---|---|---|
| 1 | control | 200 Hz | the ONLY writer of motors; applies line PID or the drive mailbox |
| 1 | line | 200 Hz | TCR5000 sweep, publishes line fields |
| 0 | sensors | 33 Hz | IMU, ToF, colour; publishes the rest of the snapshot |
| 0 | mission | 50 Hz | state machine, PCA9685 (servos/LED/buzzer) |
| 0 | telemetry | 5 Hz | serial diagnostics (optional) |

### Rules of the road (do not break these)
1. **Only the control task writes motors.** The mission task posts one packed 32-bit word
   (`g_driveCmd`: command + left + right) and the control task applies it. Post the command
   BEFORE clearing `g_lineFollowEnabled` (`stopDriving()` does this).
2. **Core 1 never touches I2C.** I2C users: sensor task (IMU+ToF) and mission task (PCA9685).
   `TwoWire` serialises transactions.
3. **No `delay()` in running tasks.** Only boot-time bring-up may block.
4. **SensorSnapshot is split by owner.** `publish()` (Core 0) writes inertial/ranging/colour
   fields; `publishLine()` (Core 1) writes only line fields. Never cross them.
5. **Distance decisions use encoder mm, not time** (marker windows, red/green runs, line-lost).
6. **Every sequence has a watchdog and bounded retries.** No unbounded loops.
7. Do not re-add `ESP32Servo`, servo LEDC channels or buzzer LEDC. They were removed on
   purpose; servos/LED/buzzer live on the PCA9685. Motor LEDC (channels 0/1) stays native.
8. Active buzzer = fixed pitch. `Indicator::beep(ms)` only; there is no tone/frequency.
9. Pivot effort is `Tune::SPEED_PIVOT` (420). `Hw::PWM_MIN_MOVE` (220) is the stiction floor;
   anything below it is silently raised to it.

## Mission summary (what the state machine does)
Calibrate (line sweep -> gyro bias -> teach WHITE/BLACK/GREEN/RED by button) -> idle armed ->
START -> follow line. At intersections: green markers latch over a distance window and the
decision is made AT the bar (both = U-turn, one side = that side, none = straight; markers
after a bar are ignored). Obstacle: first verify it is not the victim (green), else bypass.
Red tile (>40 mm run) = mine: park chassis centre on tile centre, 5 s dwell with LED dark,
3 LED blinks (250/250 ms), buzzer 600 ms, then ignore red for 200 mm. Green tile =
victim zone: stop-and-go scan (±70°, 8° steps), align on IMU, approach, lower arm at
110 mm, grip, verify, lift, U-turn. Carrying: red run <= 60 mm = evacuation stripe ->
hold 5 s -> finish; red run >= 120 mm = mine, defuse and continue.
Retries bounded (3), then LACK_OF_PROGRESS (wait for button).
**Cross-check these numbers and behaviours against the rulebook PDF** and tell the user
about any mismatch rather than silently changing scoring logic.

## Constants the user must MEASURE on the real robot (do not guess)
`Config.h`: `Hw::LINE_ARRAY_TO_PIVOT_MM`, `Hw::COLOR_HEADS_TO_CENTER_MM`,
`Hw::COLOR_HEADS_BEHIND_ARRAY_MM`, `Hw::IMU_YAW_SIGN` (turn robot left by hand, yaw must go
positive), `Hw::TRACK_WIDTH_MM`, `Hw::WHEEL_DIAMETER_MM`, `Hw::ENCODER_CPR`,
`Hw::PCA_OSC_HZ` (calibrate by measuring one servo pulse), servo angles
(`ARM_ANGLE_*`, `GRIPPER_*`), `Tune::LINE_KP/KI/KD` (KD=12 is only a start; tuning
procedure is in the comment), `Mission::OBSTACLE_*` geometry for the real chassis,
`Tune::TOF_*`. Ask the user for the measurements; never invent them.

## Host tests (no hardware needed)
`host_tests/run.sh` (needs g++ with C++17; on Windows use WSL or MSYS2). Two suites:
- `lidar_sim`: XSHUT addressing (cold boot, warm reset, stuck XSHUT, dead side sensor).
- `mission_sim`: compiles the REAL `TaskHandler.cpp` against stubs and runs scenarios
  (intersections, markers, exit stripe, mine re-arm, obstacle, victim, retries, IMU loss,
  PCA9685 brown-out recovery, slew limiter equivalence, mine LED/buzzer order).
After any change to `src/`, run them. If a test must change because behaviour changed on
purpose, say so explicitly.
They are NOT a substitute for `pio run` or for testing on the robot.

## Working style
- Small, reviewable changes. Show the user what you changed and why, in Arabic.
- Never delete the user's files without asking. Do not commit unless asked.
- If something needs the physical robot (a measurement, a wiring check, a servo pulse),
  stop and tell the user exactly what to do and what number to bring back.
- Prefer fixing root causes over adding `#ifdef` workarounds.
