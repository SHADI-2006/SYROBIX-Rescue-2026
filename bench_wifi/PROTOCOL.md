# SYROBIX Bench — Serial protocol

- **Link:** USB serial, **460800 baud**, 8N1. On the DevKitC use the **UART/COM** USB port
  (`Serial` = UART0, same as the match firmware's default).
- **Host → device:** one ASCII command per line, terminated by `\n` (`\r` is ignored).
  Tokens are separated by spaces, case-insensitive. Max line length 159 chars.
- **Device → host:** one JSON object per line. Two kinds:
  - telemetry: `{"ty":"tl", ...}` every 50 ms (~20 Hz, change with `TELE`)
  - events: `{"ev":"<name>", ...}` (replies, test results, warnings)

## Safety rules enforced by the device

| Rule | Value |
|---|---|
| Boot state | STBY LOW, motor PWM 0, all 16 PCA9685 channels full-OFF (servos get no pulse) |
| Motor deadman | an `M` command lives **400 ms**. Re-send every 150 ms or the motors stop (`{"ev":"deadman"}`) |
| Sequence heartbeat | `DIR` / `DB` abort if no line arrives for 400 ms. Send `HB` every 150 ms |
| PWM cap | default **40 %** of `Hw::PWM_MAX`. `CAP <pct>` above 40 needs the token `YES` |
| Servo limits | soft limits default to the Config.h pose range (arm `ARM_ANGLE_DOWN..ARM_ANGLE_CARRY`, gripper `GRIPPER_CLOSED..GRIPPER_OPEN`). Wider needs `YES`. Hard clamp `0..SERVO_MAX_ANGLE` |
| Servo motion | slewed at `SLEW` deg/s (default 140, as TaskHandler.cpp). Never moves at boot |
| Buzzer | `OUT B1 1` auto-offs after 3 s. `BEEP` max 3000 ms |
| `STOP` | motors off + STBY LOW, sequences aborted, servos limp (no pulse), buzzer and LED off |

## Motor sign convention

`M <left> <right>` takes **firmware convention**: + is "forward" as the match firmware believes
it, i.e. after the inversions hard-coded in `src/RobotDrivetrain.cpp`
(`_right(..., true)`, `_encRight(..., true)`). Telemetry reports both:
`m.fw` (what you asked) and `m.raw` (what the pins got, + = IN1 HIGH);
`e.raw` (encoder as wired) and `e.fw` (with the firmware inversion).
The direction test is what tells you whether those inversions match your wiring.

**No stiction floor** is applied (unlike `MotorSide::setPwm()`): the bench shows the truth.

## Commands

| Command | Meaning | Reply |
|---|---|---|
| `HELLO` | re-send config + boot report | `hello` |
| `HB` | heartbeat (no reply) | — |
| `STOP` | stop everything | `stop` |
| `M <l> <r>` | motor PWM per side, firmware convention, ±PWM_MAX, clamped to cap. Lives 400 ms | — |
| `CAP <pct> [YES]` | PWM cap 5..100 %. >40 needs `YES` | `cap` |
| `ENC RESET` | zero both encoder counters | `ack` |
| `DIR <L\|R> [pwm]` | 500 ms forward pulse on one side (default pwm = `PWM_MIN_MOVE`+60), then 300 ms coast | `dir` |
| `DB <L\|R\|P> [+\|-]` | ramp PWM 0→cap in steps of 8 every 80 ms until the encoder moves ≥10 counts. `P` = pivot left (L back, R forward) | `db` |
| `IMU INIT` | re-run MPU bring-up | `imu_init` |
| `IMU STILL [ms]` | stillness test (default 3000, 500..10000). Also stores the gyro bias and zeros yaw | `imu_still` |
| `IMU ZERO` | yaw := 0 | `ack` |
| `TOF INIT` | XSHUT re-addressing, same steps as LidarBank::begin() | `tof_init` |
| `TOF STAT <id> [ms]` | statistics of every new valid sample for `ms` (default 2000). id 1=front 2=left 3=right | `tof_stat` |
| `TOF CFG <id> <S\|M\|L> <budget_ms>` | change distance mode + timing budget (min 20 ms SHORT, 33 ms MEDIUM/LONG) | `ack` |
| `I2C SCAN` | list every address that ACKs | `i2c` |
| `PCA INIT` / `PCA STAT` | re-init / read MODE1 + PRESCALE | `pca` |
| `PCA OSC <hz>` | try another oscillator value (20e6..30e6) and re-init | `pca` |
| `SV <1\|2> ON <deg>` | enable servo at this angle (first pulse jumps there: real position is unknown) | `ack` |
| `SV <1\|2> <deg>` | new target, slewed (no reply) | — |
| `SV <1\|2> OFF` | no pulse (limp) | `ack` |
| `SLIM <1\|2> <lo> <hi> [YES]` | soft limits. Outside the Config.h pose range needs `YES` | `ack` |
| `SLEW <deg_s>` | servo slew rate 10..360 | `ack` |
| `OUT D1 <0\|1>` | status LED | `ack` |
| `OUT B1 <0\|1>` | buzzer (auto-off after 3 s) | `ack` |
| `BEEP <ms>` | buzzer pulse 1..3000 ms | `ack` |
| `COL <L\|R\|B\|OFF>` | which colour head(s) to sample | `ack` |
| `TELE <ms>` | telemetry period 20..1000 | `ack` |

Errors: `{"ev":"err","cmd":"M","msg":"usage: ..."}`.

## Telemetry fields (`ty":"tl"`)

```json
{"ty":"tl","t":12345,"loop":850,
 "m":{"fw":[0,0],"raw":[0,0],"stby":0,"cap":409,"pct":40,"seq":"none"},
 "e":{"raw":[0,0],"fw":[0,0],"bad":[0,0]},
 "l":[812,790,...8 values 0..4095],
 "g":{"ok":1,"who":112,"cfg":1,"g":[dps x,y,z],"a":[g x,y,z],"yaw":0.0,"roll":0.0,"pitch":0.0,
      "tC":31.2,"bias":1,"still":0,"frz":0,"zero":0,"ff":0,"err":0},
 "tof":[{"up":1,"f":0,"mm":120,"st":0,"sig":12.5,"amb":0.3,"age":20,"n":530,"err":0,"mode":0,"bud":33,"stat":0}, x3],
 "p":{"ack":1,"up":1,"m1":32,"pre":131,"exp":131,"osc":27000000,"init":1,"rst":0,"wf":0},
 "s":[{"en":0,"cur":10.0,"tgt":10,"lo":10,"hi":75,"tk":0}, x2],"slew":140,"d1":0,"b1":0,
 "c":{"sel":3,"f":120,"L":[r,g,b,c],"R":[r,g,b,c]},
 "btn":{"p":0,"n":0},"st":0}
```

- `loop`: worst loop time (µs) since the previous telemetry line.
- `e.bad`: illegal quadrature transitions (both lines changed at once): noise or missed edges.
- `g.g` is bias-corrected; `g.yaw` is the integral of `g.g[2]` with the firmware's 0.35 dps deadband,
  **raw sign** (Hw::IMU_YAW_SIGN is NOT applied), unwrapped.
- `g.frz` = the same 14 bytes 20 times in a row; `g.zero` = all axes 0; `g.ff` = all axes −1 (bus floating).
- `tof[i].mm` is **raw**: `Tof::OFFSET_MM` is not added. `st` = Pololu `RangeStatus`
  (0 valid, 1 sigma, 2 signal, 3 min-range clipped (accepted by LidarBank), 4 out of bounds,
  5 hardware, 6 no wrap check, 7 wrap, 255 none). `f` = bring-up result:
  0 ok, 1 bus stuck (answered with all XSHUT low), 2 no ACK at 0x29 after release,
  3 init failed, 4 address change failed, 5 config rejected, 6 not tried.
- `p.exp` = prescale Adafruit computes for `p.osc` at 50 Hz. `p.rst` counts detected resets
  (MODE1 SLEEP set or prescale changed) — each is a likely servo brown-out.
- `s[i].tk` = 12-bit ticks written (same map as TaskHandler.cpp: 0°→102, 180°→491).
- `c.L/R` = intensity `500000 / halfPeriod_us` per filter (as ColorVision), 0 = timeout.
- `st` self-test state: 0 idle, 1 countdown, 2 running, 3 done.

## Events

| `ev` | Fields |
|---|---|
| `hello` | `fw`, `ver`, `cfg{...Config.h values...}`, `boot{imu_ack, who, imu_cfg, tof_fail[3], tof_stuck, pca_ack, pca_up, i2c[]}` |
| `imu_still` | `n`, `ms`, `frozen`, `g_mean[3]`, `g_std[3]` (dps), `a_mean[3]`, `a_std[3]` (g), `a_norm` |
| `tof_stat` | `id`, `n`, `bad`, `mean`, `std`, `min`, `max`, `ms` |
| `dir` | `side`, `pwm`, `d_raw[2]`, `d_fw[2]`, `inv_mot[2]`, `inv_enc[2]` |
| `db` | `mode` (L/R/P), `dir`, `move[2]` (PWM where each side started moving, −1 = never), `cap`, `step`, `result` (`moved`/`cap`) |
| `seq` | `state:"aborted"`, `why` |
| `deadman` | motors stopped because no `M` for 400 ms |
| `selftest` | `state`: countdown / done / aborted |
| `i2c` | `found[]` decimal addresses |
| `pca` | `ack`, `up`, `mode1`, `prescale`, `expected`, `osc`, `inits` |
| `cap` | `pct`, `pwm` |
| `stop` | `why` |
| `ack` / `err` | `cmd` (+ `msg`) |

---

## Wi-Fi edition (bench_wifi) — additions

- **Transport:** WebSocket `ws://<robot>:81/`, one command per text frame (or several separated
  by `\n`). Every device line is broadcast as one text frame. USB serial keeps working in parallel.
  The UI is served at `http://<robot>/` (gzipped, embedded at build time from `bench/web/index.html`).
- **Default network:** the robot's own access point `SYROBIX-BENCH`, address `192.168.4.1`
  (see `src/wifi_config.h`).
- `hello` gains `"mode":"bench"|"run"`, `"ip"`, `"net":"ap"|"sta"`.
- The UI sends `TELE 100` after connecting over Wi-Fi (10 Hz).
- Extra safety: the last WebSocket client leaving stops motors and sequences at once
  (`{"ev":"deadman","why":"wifi client gone"}`); a 50 ms esp_timer drops STBY if the host is
  silent for > 500 ms even when `loop()` is blocked (`"why":"timer"`).

### Bench-mode command

| Command | Meaning | Reply |
|---|---|---|
| `MODE RUN` | stop everything, store a one-shot flag, reboot into RUN mode | `{"ev":"restarting","mode":"run"}` |

### RUN mode (real match firmware)

Commands: `HB`, `HELLO`, `START` (virtual START press, 200 ms open-drain LOW on GPIO0),
`STOP` (`TaskHandler::emergencyStop()`), `LINKSTOP <0|1>` (stop the robot if no line arrives for
1500 ms while it is moving), `MODE RUN` (restart the practice), `MODE BENCH`.

Status, 5 Hz:

```json
{"ty":"run","t":12345,"st":3,"name":"LINE_FOLLOW","prev":"IDLE_ARMED","in":4200,"match":4200,
 "zone":0,"mines":0,"vic":0,"deliv":0,"att":0,"lop":0,"score":0,
 "estop":0,"linkstopped":0,"userstop":0,"btn":0,"snap":1,
 "line":0.12,"lost":0,"act":2,"yaw":-3.1,"pitch":0.4,"ramp":0,"imu":1,
 "fr":620,"lr":300,"rr":280,"cl":1,"cr":1,"fault":""}
```

`name` is the `RobotState` name; `match` = ms since `MissionContext::matchStartMs` (0 = not started);
`cl/cr` = `TileColor` (0 unknown, 1 white, 2 black, 3 green, 4 red, 5 silver). If the match firmware
failed to boot, only `{"ty":"run","name":"BOOT_FAULT","fault":"<reason>"}` is sent.
