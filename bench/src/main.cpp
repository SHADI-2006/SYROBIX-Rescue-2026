/**
 * ============================================================================
 *  @file       main.cpp
 *  @project    SYROBIX Bench Tool  |  ESP32-S3
 *  @brief      Standalone hardware test firmware. Talks to every part of the
 *              robot DIRECTLY (analogRead, Wire, Pololu VL53L1X, Adafruit
 *              PCA9685) — none of the match-firmware classes are used, so a
 *              "this sensor is dead" verdict is about the hardware, not about
 *              a bug in LineArray / LidarBank / SensorsSystem.
 *
 *  Pins, addresses and limits come from ../src/Config.h (a pure constants
 *  header). The few constants the match firmware keeps OUTSIDE Config.h are
 *  copied below, each with a pointer to where it lives.
 *
 *  Protocol: see ../PROTOCOL.md. ASCII command lines in, one JSON object per
 *  line out, 460800 baud.
 *
 *  SAFETY (all mandatory)
 *    - Boot: STBY LOW, motor PWM 0, every PCA9685 channel full-OFF. Servos
 *      receive no pulse until the host explicitly enables them.
 *    - Motor deadman: a motor command lives MOTOR_DEADMAN_MS. The web UI
 *      re-sends every 150 ms; if the link or the page dies, the motors stop.
 *    - Device-run sequences (direction test, deadband finder) need a host
 *      heartbeat (any line, "HB" every 150 ms) or they abort.
 *    - PWM cap 40 % by default; raising it needs an explicit "YES" token.
 *    - STOP (command, or the UI button / Space key) kills everything.
 *    - No Wi-Fi, no Bluetooth. Nothing here touches the mission logic.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "Config.h"   // ../src/Config.h — the match firmware's single source of truth

// ============================================================================
//  SECTION A — BENCH CONSTANTS (test settings, NOT robot measurements)
// ============================================================================

static constexpr uint32_t BENCH_BAUD           = 460800;
static constexpr uint32_t TELEMETRY_DEFAULT_MS = 50;      // ~20 Hz
static constexpr uint32_t MOTOR_DEADMAN_MS     = 400;     // a motor command lives this long
static constexpr uint32_t HOST_DEADMAN_MS      = 400;     // sequences abort without a heartbeat
static constexpr uint8_t  PWM_CAP_DEFAULT_PCT  = 40;      // above this needs "YES"
static constexpr uint32_t IMU_PERIOD_US        = 5000;    // 200 Hz, matches SMPLRT_DIV below
static constexpr uint32_t LINE_PERIOD_US       = 5000;    // 200 Hz, like the line task
static constexpr uint16_t IMU_FROZEN_SAMPLES   = 20;      // identical 14-byte bursts in a row
static constexpr uint16_t BEEP_MAX_MS          = 3000;

// --- Direction test: one low-power pulse on one side ----------------------
static constexpr uint16_t DIR_PULSE_MS         = 500;
static constexpr uint16_t DIR_SETTLE_MS        = 300;
/** Test effort. Just above the firmware's stiction floor so the wheel turns. */
static constexpr int16_t  DIR_TEST_PWM_DEFAULT = Hw::PWM_MIN_MOVE + 60;

// --- Deadband finder: ramp PWM until the encoder moves ---------------------
static constexpr int16_t  DB_STEP_PWM          = 8;       // resolution of the answer
static constexpr uint16_t DB_STEP_MS           = 80;
static constexpr int32_t  DB_MOVE_COUNTS       = 10;      // ~0.7 % of a wheel turn at 1440 CPR

// --- Optional self-test without a laptop (BTN after boot) -----------------
//  Armed only if NO host command has been received since boot. Press START:
//  5 s countdown with a beep per second, then a short low-power motor
//  sequence (left fwd, left back, right fwd, right back), then stop. Pressing
//  START again (or any serial command) aborts it immediately.
#define BENCH_SELFTEST_ENABLED 1
static constexpr uint32_t SELFTEST_COUNTDOWN_MS = 5000;
static constexpr uint16_t SELFTEST_BEEP_MS      = 80;
static constexpr int16_t  SELFTEST_PWM          = Hw::PWM_MIN_MOVE + 60;   // low; test value
static constexpr uint16_t SELFTEST_STEP_MS      = 400;
static constexpr uint16_t SELFTEST_GAP_MS       = 300;

static_assert(SELFTEST_PWM <= (Hw::PWM_MAX * PWM_CAP_DEFAULT_PCT) / 100,
              "Self-test PWM must stay under the default 40 % cap.");
static_assert(DIR_TEST_PWM_DEFAULT <= (Hw::PWM_MAX * PWM_CAP_DEFAULT_PCT) / 100,
              "Direction-test PWM must stay under the default 40 % cap.");

// ============================================================================
//  SECTION B — FIRMWARE CONSTANTS THAT ARE NOT IN Config.h (copied)
// ============================================================================

// src/RobotDrivetrain.cpp, RobotDrivetrain::RobotDrivetrain() initialiser list:
//   _left (..., false), _right (..., true)  "mirrored mounting"
//   _encLeft(..., false), _encRight(..., true)
static constexpr bool FW_INVERT_MOTOR[2] = { false, true };
static constexpr bool FW_INVERT_ENC[2]   = { false, true };

// src/RobotDrivetrain.cpp, QUAD_LUT — index = (prev << 2) | curr, state = (A << 1) | B.
static constexpr int8_t QUAD_LUT[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// src/TaskHandler.cpp, PcaBus — servo angle -> 12-bit ticks, and slew rate.
static constexpr long  SERVO_MIN_TICKS   = 102;
static constexpr long  SERVO_MAX_TICKS   = 491;
static constexpr float SERVO_DEG_PER_SEC = 140.0f;
static constexpr uint8_t PCA_REG_MODE1   = 0x00;
static constexpr uint8_t PCA_REG_PRESCALE = 0xFE;
static constexpr uint8_t PCA_REG_ALL_OFF_H = 0xFD;   // ALL_LED_OFF_H (datasheet)
static constexpr uint8_t PCA_MODE1_SLEEP = 0x10;
static constexpr uint16_t PCA_FULL_BIT   = 4096;

// src/SensorsSystem.cpp — MPU6500 register map and scaling.
static constexpr uint8_t MPU_SMPLRT_DIV   = 0x19;
static constexpr uint8_t MPU_CONFIG       = 0x1A;
static constexpr uint8_t MPU_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t MPU_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t MPU_WHO_AM_I     = 0x75;
static constexpr uint8_t MPU_WHOAMI_6500  = 0x70;
static constexpr float   GYRO_LSB_PER_DPS = 65.5f;    // +/-500 dps
static constexpr float   ACCEL_LSB_PER_G  = 8192.0f;  // +/-4 g
static constexpr float   GYRO_DEADBAND_DPS = 0.35f;

// src/SensorsSystem.cpp — TCS3200 timing.
static constexpr uint32_t COLOR_PULSE_TIMEOUT_US = 3000;
static constexpr uint16_t COLOR_SETTLE_US        = 120;

// src/LineArray.cpp — reported to the UI so its verdict matches the firmware.
static constexpr uint16_t LINE_MIN_USABLE_SPREAD = 300;
static constexpr bool     LINE_INVERT_POLARITY   = false;

// ============================================================================
//  SECTION C — OUTPUT HELPERS
// ============================================================================

namespace {

/** Line builder: one JSON object per Serial line. */
struct Line {
    char   b[1800];
    size_t n = 0;
    void add(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    void send() {
        Serial.write(reinterpret_cast<const uint8_t*>(b), n);
        Serial.write('\n');
        n = 0;
    }
};

void Line::add(const char* fmt, ...) {
    if (n >= sizeof(b) - 1) return;
    va_list ap;
    va_start(ap, fmt);
    const int w = vsnprintf(b + n, sizeof(b) - n, fmt, ap);
    va_end(ap);
    if (w < 0) return;
    n += static_cast<size_t>(w);
    if (n > sizeof(b) - 1) n = sizeof(b) - 1;
}

Line g_line;

void emit(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void emit(const char* fmt, ...) {
    char buf[600];
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (w < 0) return;
    if (w >= static_cast<int>(sizeof(buf))) w = sizeof(buf) - 1;
    Serial.write(reinterpret_cast<const uint8_t*>(buf), static_cast<size_t>(w));
    Serial.write('\n');
}

void emitErr(const char* cmd, const char* msg) {
    emit("{\"ev\":\"err\",\"cmd\":\"%s\",\"msg\":\"%s\"}", cmd, msg);
}

void emitAck(const char* cmd) {
    emit("{\"ev\":\"ack\",\"cmd\":\"%s\"}", cmd);
}

/** JSON cannot carry NaN/Inf. */
inline float fin(float v) { return isfinite(v) ? v : 0.0f; }

bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t& out) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) return false;
    out = static_cast<uint8_t>(Wire.read());
    return true;
}

bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// ============================================================================
//  SECTION D — HOST LINK STATE
// ============================================================================

uint32_t g_lastHostMs   = 0;
bool     g_hostSeen     = false;    // any command since boot -> self-test disarmed
uint32_t g_teleMs       = TELEMETRY_DEFAULT_MS;

bool hostAlive(uint32_t now) {
    return g_hostSeen && (now - g_lastHostMs) <= HOST_DEADMAN_MS;
}

// ============================================================================
//  SECTION E — MOTORS (TB6612FNG x2, sides ganged) + deadman + cap
// ============================================================================

const uint8_t kMotPwm[2] = { Pins::MOTOR_PWM_LEFT,  Pins::MOTOR_PWM_RIGHT };
const uint8_t kMotIn1[2] = { Pins::MOTOR_LEFT_IN1,  Pins::MOTOR_RIGHT_IN1 };
const uint8_t kMotIn2[2] = { Pins::MOTOR_LEFT_IN2,  Pins::MOTOR_RIGHT_IN2 };
const uint8_t kMotCh[2]  = { Hw::LEDC_CH_MOTOR_LEFT, Hw::LEDC_CH_MOTOR_RIGHT };

int16_t  g_capPwm     = (Hw::PWM_MAX * PWM_CAP_DEFAULT_PCT) / 100;
uint8_t  g_capPct     = PWM_CAP_DEFAULT_PCT;
int16_t  g_motFw[2]   = { 0, 0 };   // firmware convention (+ = forward per FW_INVERT_MOTOR)
int16_t  g_motRaw[2]  = { 0, 0 };   // what the pins get (+ = IN1 HIGH)
uint32_t g_motorCmdMs = 0;
bool     g_stby       = false;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define BENCH_LEDC_V3 1
#else
  #define BENCH_LEDC_V3 0
#endif

void ledcOut(uint8_t side, uint32_t duty) {
#if BENCH_LEDC_V3
    ledcWrite(kMotPwm[side], duty);
#else
    ledcWrite(kMotCh[side], duty);
#endif
}

void setStby(bool on) {
    digitalWrite(Pins::MOTOR_STBY, on ? HIGH : LOW);
    g_stby = on;
}

void writeSideRaw(uint8_t side, int16_t raw) {
    if (raw >  g_capPwm) raw =  g_capPwm;
    if (raw < -g_capPwm) raw = -g_capPwm;
    g_motRaw[side] = raw;
    if (raw == 0) {
        ledcOut(side, 0);
        digitalWrite(kMotIn1[side], LOW);
        digitalWrite(kMotIn2[side], LOW);
        return;
    }
    // NO stiction floor here on purpose: the bench shows the raw truth.
    digitalWrite(kMotIn1[side], raw > 0 ? HIGH : LOW);
    digitalWrite(kMotIn2[side], raw > 0 ? LOW  : HIGH);
    ledcOut(side, static_cast<uint32_t>(raw > 0 ? raw : -raw));
}

/** Firmware convention in, raw out. STBY follows "any side non-zero". */
void setMotorsFw(int16_t left, int16_t right) {
    const int16_t fw[2] = { left, right };
    for (uint8_t s = 0; s < 2; ++s) {
        int16_t v = fw[s];
        if (v >  g_capPwm) v =  g_capPwm;
        if (v < -g_capPwm) v = -g_capPwm;
        g_motFw[s] = v;
        writeSideRaw(s, FW_INVERT_MOTOR[s] ? static_cast<int16_t>(-v) : v);
    }
    setStby(g_motRaw[0] != 0 || g_motRaw[1] != 0);
}

void motorsOff() {
    setStby(false);          // drivers off FIRST
    for (uint8_t s = 0; s < 2; ++s) {
        g_motFw[s] = 0;
        writeSideRaw(s, 0);
    }
}

void motorsBegin() {
    pinMode(Pins::MOTOR_STBY, OUTPUT);
    digitalWrite(Pins::MOTOR_STBY, LOW);
    for (uint8_t s = 0; s < 2; ++s) {
        digitalWrite(kMotIn1[s], LOW);
        digitalWrite(kMotIn2[s], LOW);
        pinMode(kMotIn1[s], OUTPUT);
        pinMode(kMotIn2[s], OUTPUT);
#if BENCH_LEDC_V3
        ledcAttachChannel(kMotPwm[s], Hw::PWM_FREQ_HZ, Hw::PWM_RESOLUTION, kMotCh[s]);
#else
        ledcSetup(kMotCh[s], Hw::PWM_FREQ_HZ, Hw::PWM_RESOLUTION);
        ledcAttachPin(kMotPwm[s], kMotCh[s]);
#endif
        ledcOut(s, 0);
    }
    motorsOff();
}

// ============================================================================
//  SECTION F — ENCODERS (x4 quadrature, raw counts)
// ============================================================================

volatile int32_t  g_encRaw[2]  = { 0, 0 };
volatile uint32_t g_encBad[2]  = { 0, 0 };   // illegal transitions (missed edges / noise)
volatile uint8_t  g_encPrev[2] = { 0, 0 };
portMUX_TYPE      g_encMux     = portMUX_INITIALIZER_UNLOCKED;

const uint8_t kEncA[2] = { Pins::ENCODER_LEFT_A, Pins::ENCODER_RIGHT_A };
const uint8_t kEncB[2] = { Pins::ENCODER_LEFT_B, Pins::ENCODER_RIGHT_B };

inline void IRAM_ATTR encEdge(uint8_t i) {
    const uint8_t cur  = static_cast<uint8_t>((digitalRead(kEncA[i]) << 1) | digitalRead(kEncB[i]));
    const uint8_t prev = g_encPrev[i];
    if (cur == prev) return;
    const int8_t d = QUAD_LUT[(prev << 2) | cur];
    g_encPrev[i] = cur;
    portENTER_CRITICAL_ISR(&g_encMux);
    if (d != 0) g_encRaw[i] = g_encRaw[i] + d;
    else        g_encBad[i] = g_encBad[i] + 1;
    portEXIT_CRITICAL_ISR(&g_encMux);
}
void IRAM_ATTR encIsrL() { encEdge(0); }
void IRAM_ATTR encIsrR() { encEdge(1); }

int32_t encRaw(uint8_t i) {
    portENTER_CRITICAL(&g_encMux);
    const int32_t v = g_encRaw[i];
    portEXIT_CRITICAL(&g_encMux);
    return v;
}
int32_t encFw(uint8_t i) { return FW_INVERT_ENC[i] ? -encRaw(i) : encRaw(i); }

void encReset() {
    portENTER_CRITICAL(&g_encMux);
    g_encRaw[0] = 0; g_encRaw[1] = 0;
    g_encBad[0] = 0; g_encBad[1] = 0;
    portEXIT_CRITICAL(&g_encMux);
}

void encBegin() {
    for (uint8_t i = 0; i < 2; ++i) {
        pinMode(kEncA[i], INPUT_PULLUP);
        pinMode(kEncB[i], INPUT_PULLUP);
        g_encPrev[i] = static_cast<uint8_t>((digitalRead(kEncA[i]) << 1) | digitalRead(kEncB[i]));
    }
    attachInterrupt(digitalPinToInterrupt(kEncA[0]), encIsrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncB[0]), encIsrL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncA[1]), encIsrR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncB[1]), encIsrR, CHANGE);
    encReset();
}

// ============================================================================
//  SECTION G — LINE SENSORS (8x TCR5000 on ADC1, raw)
// ============================================================================

uint16_t g_lineRaw[8] = { 0 };
uint32_t g_lineLastUs = 0;

void lineBegin() {
    analogReadResolution(12);
    for (uint8_t i = 0; i < 8; ++i) {
        pinMode(Pins::LINE_SENSOR[i], INPUT);
        analogSetPinAttenuation(Pins::LINE_SENSOR[i], ADC_11db);   // as LineArray::begin()
    }
}

void lineService() {
    const uint32_t nowUs = micros();
    if (nowUs - g_lineLastUs < LINE_PERIOD_US) return;
    g_lineLastUs = nowUs;
    for (uint8_t i = 0; i < 8; ++i) {
        g_lineRaw[i] = static_cast<uint16_t>(analogRead(Pins::LINE_SENSOR[i]));
    }
}

// ============================================================================
//  SECTION H — I2C SCAN
// ============================================================================

uint8_t g_scan[24];
uint8_t g_scanN = 0;

void i2cScan() {
    g_scanN = 0;
    for (uint8_t a = 0x08; a < 0x78 && g_scanN < sizeof(g_scan); ++a) {
        if (i2cProbe(a)) g_scan[g_scanN++] = a;
    }
}

void emitScan() {
    g_line.add("{\"ev\":\"i2c\",\"found\":[");
    for (uint8_t i = 0; i < g_scanN; ++i) g_line.add("%s%u", i ? "," : "", g_scan[i]);
    g_line.add("]}");
    g_line.send();
}

// ============================================================================
//  SECTION I — IMU (MPU6500, register level)
// ============================================================================

struct ImuState {
    bool     ack        = false;   // answered WHO_AM_I
    uint8_t  who        = 0;
    bool     cfgOk      = false;   // GYRO_CONFIG read back as written
    bool     present    = false;
    int16_t  raw[7]     = { 0 };   // ax ay az temp gx gy gz
    uint8_t  prevBurst[14] = { 0 };
    uint16_t sameCount  = 0;
    bool     frozen     = false;
    bool     zeros      = false;
    bool     ones       = false;   // all 0xFFFF: bus floating
    uint32_t readErr    = 0;
    float    g[3]       = { 0 };   // dps, bias removed
    float    a[3]       = { 0 };   // g
    float    tempC      = 0;
    float    bias[3]    = { 0 };   // dps
    bool     biasSet    = false;
    double   yaw        = 0;       // deg, unwrapped, RAW sign (no IMU_YAW_SIGN)
    float    roll       = 0, pitch = 0;
    uint32_t lastUs     = 0;
    // stillness test
    bool     stillRun   = false;
    uint32_t stillEndMs = 0, stillStartMs = 0;
    uint32_t stillN     = 0;
    uint32_t stillFrozen = 0;
    double   sg[3] = { 0 }, sgg[3] = { 0 }, sa[3] = { 0 }, saa[3] = { 0 };
} g_imu;

bool imuBegin() {
    ImuState& m = g_imu;
    m.ack = i2cReadReg(Hw::MPU6500_ADDR, MPU_WHO_AM_I, m.who);
    m.present = false;
    m.cfgOk = false;
    if (!m.ack) return false;
    // Same bring-up as Imu::begin(), but applied whatever WHO_AM_I says so a
    // lookalike chip can still be inspected (the UI flags the mismatch).
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_PWR_MGMT_1, 0x80);
    delay(100);
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_PWR_MGMT_1, 0x01);
    delay(10);
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_CONFIG,       0x03);   // DLPF 41 Hz
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_SMPLRT_DIV,   0x04);   // 200 Hz
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_GYRO_CONFIG,  0x08);   // +/-500 dps
    i2cWriteReg(Hw::MPU6500_ADDR, MPU_ACCEL_CONFIG, 0x08);   // +/-4 g
    uint8_t rb = 0;
    m.cfgOk = i2cReadReg(Hw::MPU6500_ADDR, MPU_GYRO_CONFIG, rb) && (rb & 0x18) == 0x08;
    m.present = true;
    m.sameCount = 0;
    m.frozen = m.zeros = m.ones = false;
    m.lastUs = micros();
    return true;
}

void imuStillStart(uint32_t ms) {
    ImuState& m = g_imu;
    m.stillRun = true;
    m.stillStartMs = millis();
    m.stillEndMs = m.stillStartMs + ms;
    m.stillN = 0;
    m.stillFrozen = 0;
    for (uint8_t i = 0; i < 3; ++i) { m.sg[i] = m.sgg[i] = m.sa[i] = m.saa[i] = 0; }
}

void imuStillFinish() {
    ImuState& m = g_imu;
    m.stillRun = false;
    const double n = m.stillN;
    float gm[3] = { 0 }, gs[3] = { 0 }, am[3] = { 0 }, as[3] = { 0 };
    if (m.stillN >= 2) {
        for (uint8_t i = 0; i < 3; ++i) {
            gm[i] = static_cast<float>(m.sg[i] / n);
            gs[i] = static_cast<float>(sqrt(fmax(0.0, m.sgg[i] / n - (m.sg[i] / n) * (m.sg[i] / n))));
            am[i] = static_cast<float>(m.sa[i] / n);
            as[i] = static_cast<float>(sqrt(fmax(0.0, m.saa[i] / n - (m.sa[i] / n) * (m.sa[i] / n))));
            m.bias[i] = gm[i];
        }
        m.biasSet = true;
        m.yaw = 0;
    }
    const float an = sqrtf(am[0] * am[0] + am[1] * am[1] + am[2] * am[2]);
    emit("{\"ev\":\"imu_still\",\"n\":%lu,\"ms\":%lu,\"frozen\":%lu,"
         "\"g_mean\":[%.4f,%.4f,%.4f],\"g_std\":[%.4f,%.4f,%.4f],"
         "\"a_mean\":[%.4f,%.4f,%.4f],\"a_std\":[%.4f,%.4f,%.4f],\"a_norm\":%.4f}",
         static_cast<unsigned long>(m.stillN),
         static_cast<unsigned long>(millis() - m.stillStartMs),
         static_cast<unsigned long>(m.stillFrozen),
         fin(gm[0]), fin(gm[1]), fin(gm[2]), fin(gs[0]), fin(gs[1]), fin(gs[2]),
         fin(am[0]), fin(am[1]), fin(am[2]), fin(as[0]), fin(as[1]), fin(as[2]), fin(an));
}

void imuService() {
    ImuState& m = g_imu;
    if (!m.present) return;
    const uint32_t nowUs = micros();
    if (nowUs - m.lastUs < IMU_PERIOD_US) return;
    const float dt = (nowUs - m.lastUs) * 1e-6f;
    m.lastUs = nowUs;

    uint8_t burst[14];
    Wire.beginTransmission(Hw::MPU6500_ADDR);
    Wire.write(MPU_ACCEL_XOUT_H);
    bool ok = (Wire.endTransmission(false) == 0) &&
              (Wire.requestFrom(Hw::MPU6500_ADDR, static_cast<uint8_t>(14)) == 14);
    if (!ok) { ++m.readErr; return; }
    for (uint8_t i = 0; i < 14; ++i) burst[i] = static_cast<uint8_t>(Wire.read());

    // Frozen = the same 14 bytes again and again. Real sensor noise flips LSBs
    // on six axes every sample; 20 identical bursts (100 ms) cannot be noise.
    if (memcmp(burst, m.prevBurst, 14) == 0) {
        if (m.sameCount < 0xFFFF) ++m.sameCount;
    } else {
        m.sameCount = 0;
        memcpy(m.prevBurst, burst, 14);
    }
    m.frozen = m.sameCount >= IMU_FROZEN_SAMPLES;

    bool allZero = true, allOnes = true;
    for (uint8_t i = 0; i < 7; ++i) {
        m.raw[i] = static_cast<int16_t>((burst[2 * i] << 8) | burst[2 * i + 1]);
        if (i == 3) continue;   // temperature
        if (m.raw[i] != 0)  allZero = false;
        if (m.raw[i] != -1) allOnes = false;
    }
    m.zeros = allZero;
    m.ones  = allOnes;

    for (uint8_t i = 0; i < 3; ++i) {
        m.a[i] = m.raw[i] / ACCEL_LSB_PER_G;
        const float dps = m.raw[4 + i] / GYRO_LSB_PER_DPS;
        if (m.stillRun) {
            m.sg[i] += dps; m.sgg[i] += static_cast<double>(dps) * dps;
            m.sa[i] += m.a[i]; m.saa[i] += static_cast<double>(m.a[i]) * m.a[i];
        }
        m.g[i] = dps - m.bias[i];
    }
    m.tempC = m.raw[3] / 333.87f + 21.0f;   // MPU6500 datasheet
    if (m.stillRun) {
        ++m.stillN;
        if (m.frozen) ++m.stillFrozen;
        if (millis() >= m.stillEndMs) imuStillFinish();
    }

    // Yaw: same deadband as Imu::update(), but RAW sign so the UI can tell
    // you which way IMU_YAW_SIGN must point. Unwrapped so 360 shows as 360.
    float rate = m.g[2];
    if (fabsf(rate) < GYRO_DEADBAND_DPS) rate = 0.0f;
    m.yaw += static_cast<double>(rate) * dt;

    // Same attitude formulas as Imu::update().
    m.pitch = atan2f(-m.a[0], sqrtf(m.a[1] * m.a[1] + m.a[2] * m.a[2])) * 57.2957795f;
    m.roll  = atan2f(m.a[1], m.a[2]) * 57.2957795f;
}

// ============================================================================
//  SECTION J — TIME OF FLIGHT (3x VL53L1X, XSHUT re-addressing)
// ============================================================================

enum TofFail : uint8_t {
    TOF_OK = 0,
    TOF_BUS_STUCK  = 1,   // something answered while every XSHUT was LOW
    TOF_NO_ACK     = 2,   // nothing at 0x29 after releasing this XSHUT
    TOF_INIT_FAIL  = 3,   // answered but init() failed (wrong chip?)
    TOF_ADDR_FAIL  = 4,   // setAddress() not confirmed on the bus
    TOF_CFG_FAIL   = 5,   // mode/budget rejected
    TOF_NOT_TRIED  = 6
};

struct TofUnit {
    uint8_t         xshut;
    uint8_t         addr;
    TofDistanceMode mode;
    uint16_t        budget;
    uint16_t        period;
};

const TofUnit kTof[3] = {
    { Pins::TOF_XSHUT_FRONT, Hw::TOF_ADDR_FRONT, Tof::FRONT_MODE, Tof::FRONT_BUDGET_MS, Tof::FRONT_PERIOD_MS },
    { Pins::TOF_XSHUT_LEFT,  Hw::TOF_ADDR_LEFT,  Tof::SIDE_MODE,  Tof::SIDE_BUDGET_MS,  Tof::SIDE_PERIOD_MS  },
    { Pins::TOF_XSHUT_RIGHT, Hw::TOF_ADDR_RIGHT, Tof::SIDE_MODE,  Tof::SIDE_BUDGET_MS,  Tof::SIDE_PERIOD_MS  },
};

VL53L1X g_tof[3];

struct TofState {
    bool            up     = false;
    uint8_t         fail   = TOF_NOT_TRIED;
    uint16_t        mm     = 0;
    uint8_t         st     = 255;
    float           sig    = 0, amb = 0;
    uint32_t        stamp  = 0, n = 0;
    uint32_t        err    = 0;
    TofDistanceMode mode   = TofDistanceMode::SHORT;
    uint16_t        budget = 0, period = 0;
    // 2 s statistics window
    bool            statRun = false;
    uint32_t        statEnd = 0, statStart = 0;
    uint32_t        sN = 0, sBad = 0;
    double          sSum = 0, sSq = 0;
    uint16_t        sMin = 0xFFFF, sMax = 0;
} g_tofSt[3];

uint8_t g_tofStuckMask = 0;   // bit0 = 0x29 answered, bit1..3 = a target address answered

void xshutAssert(uint8_t pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

void xshutRelease(uint8_t pin) {
#if SYROBIX_TOF_XSHUT_OPEN_DRAIN
    pinMode(pin, INPUT);           // breakout pull-up raises XSHUT (Config.h note)
#else
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
#endif
}

VL53L1X::DistanceMode driverMode(TofDistanceMode m) {
    switch (m) {
        case TofDistanceMode::SHORT:  return VL53L1X::Short;
        case TofDistanceMode::MEDIUM: return VL53L1X::Medium;
        default:                      return VL53L1X::Long;
    }
}

uint16_t minBudget(TofDistanceMode m) {
    return (m == TofDistanceMode::SHORT) ? Tof::MIN_BUDGET_SHORT_MS : Tof::MIN_BUDGET_LONG_MS;
}

bool tofApply(uint8_t i, TofDistanceMode mode, uint16_t budget, uint16_t period) {
    if (budget < minBudget(mode) || budget > Tof::MAX_BUDGET_MS) return false;
    if (period < budget) period = budget + Tof::PERIOD_SLACK_MS;
    VL53L1X& d = g_tof[i];
    d.stopContinuous();
    if (!d.setDistanceMode(driverMode(mode))) return false;                          // mode first
    if (!d.setMeasurementTimingBudget(static_cast<uint32_t>(budget) * 1000UL)) return false;  // us
    if (d.last_status != 0) return false;
    d.startContinuous(period);
    g_tofSt[i].mode = mode;
    g_tofSt[i].budget = budget;
    g_tofSt[i].period = period;
    return true;
}

void tofBegin() {
    g_tofStuckMask = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        if (g_tofSt[i].up) g_tof[i].stopContinuous();
        g_tofSt[i] = TofState();
        xshutAssert(kTof[i].xshut);
    }
    delay(Tof::RESET_HOLD_MS);

    // Same refusal as LidarBank::begin(): with every XSHUT low nothing may
    // answer, or an XSHUT line is not reaching its unit.
    if (i2cProbe(Tof::DEFAULT_ADDR)) g_tofStuckMask |= 1;
    for (uint8_t i = 0; i < 3; ++i) {
        if (i2cProbe(kTof[i].addr)) g_tofStuckMask |= static_cast<uint8_t>(2u << i);
    }
    if (g_tofStuckMask) {
        for (uint8_t i = 0; i < 3; ++i) g_tofSt[i].fail = TOF_BUS_STUCK;
        return;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        TofState& s = g_tofSt[i];
        VL53L1X& d = g_tof[i];
        d = VL53L1X();
        d.setBus(&Wire);
        d.setTimeout(Tof::IO_TIMEOUT_MS);   // non-zero, or init() can hang

        xshutRelease(kTof[i].xshut);
        delay(Tof::BOOT_WAIT_MS);

        if (!i2cProbe(Tof::DEFAULT_ADDR)) {
            delay(Tof::BOOT_WAIT_MS * 5);   // one generous retry before blaming the wiring
            if (!i2cProbe(Tof::DEFAULT_ADDR)) {
                s.fail = TOF_NO_ACK;
                xshutAssert(kTof[i].xshut);
                continue;
            }
        }
        bool ok = false;
        for (uint8_t a = 0; a < Tof::INIT_RETRIES && !ok; ++a) {
            ok = d.init();
            if (!ok) delay(Tof::BOOT_WAIT_MS);
        }
        if (!ok) { s.fail = TOF_INIT_FAIL; xshutAssert(kTof[i].xshut); continue; }

        d.setAddress(kTof[i].addr);
        if (!i2cProbe(kTof[i].addr) || i2cProbe(Tof::DEFAULT_ADDR)) {
            s.fail = TOF_ADDR_FAIL; xshutAssert(kTof[i].xshut); continue;
        }
        if (!tofApply(i, kTof[i].mode, kTof[i].budget, kTof[i].period)) {
            s.fail = TOF_CFG_FAIL; xshutAssert(kTof[i].xshut); continue;
        }
        s.fail = TOF_OK;
        s.up = true;
    }
}

void tofStatFinish(uint8_t i) {
    TofState& s = g_tofSt[i];
    s.statRun = false;
    const double n = s.sN;
    const double mean = n > 0 ? s.sSum / n : 0;
    const double sd = n > 1 ? sqrt(fmax(0.0, s.sSq / n - mean * mean)) : 0;
    emit("{\"ev\":\"tof_stat\",\"id\":%u,\"n\":%lu,\"bad\":%lu,\"mean\":%.2f,\"std\":%.2f,"
         "\"min\":%u,\"max\":%u,\"ms\":%lu}",
         i + 1, static_cast<unsigned long>(s.sN), static_cast<unsigned long>(s.sBad),
         mean, sd, s.sN ? s.sMin : 0, s.sMax,
         static_cast<unsigned long>(millis() - s.statStart));
}

void tofService() {
    for (uint8_t i = 0; i < 3; ++i) {
        TofState& s = g_tofSt[i];
        if (!s.up) continue;
        VL53L1X& d = g_tof[i];
        const bool ready = d.dataReady();
        if (d.last_status != 0) { ++s.err; continue; }
        if (ready) {
            const uint16_t mm = d.read(false);
            if (d.last_status != 0) { ++s.err; continue; }
            s.mm  = mm;                   // RAW: Tof::OFFSET_MM is NOT added here
            s.st  = static_cast<uint8_t>(d.ranging_data.range_status);
            s.sig = d.ranging_data.peak_signal_count_rate_MCPS;
            s.amb = d.ranging_data.ambient_count_rate_MCPS;
            s.stamp = millis();
            ++s.n;
            if (s.statRun) {
                if (s.st == VL53L1X::RangeValid || s.st == VL53L1X::RangeValidMinRangeClipped) {
                    ++s.sN;
                    s.sSum += mm;
                    s.sSq  += static_cast<double>(mm) * mm;
                    if (mm < s.sMin) s.sMin = mm;
                    if (mm > s.sMax) s.sMax = mm;
                } else {
                    ++s.sBad;
                }
            }
        }
        if (s.statRun && millis() >= s.statEnd) tofStatFinish(i);
    }
}

// ============================================================================
//  SECTION K — PCA9685: servos, status LED, buzzer, health
// ============================================================================

Adafruit_PWMServoDriver pca(Hw::PCA9685_ADDR);

struct PcaState {
    bool     ack = false, up = false;
    uint8_t  mode1 = 0, prescale = 0, expected = 0;
    uint32_t osc = Hw::PCA_OSC_HZ;
    uint32_t inits = 0, resets = 0, writeFail = 0;
    uint32_t lastCheckMs = 0;
} g_pca;

struct ServoState {
    uint8_t  ch;
    bool     en = false;
    float    cur = 0;
    int16_t  tgt = 0;
    uint8_t  lo = 0, hi = 0;          // active soft limits
    uint8_t  cfgLo = 0, cfgHi = 0;    // Config.h pose range
    uint16_t ticks = 0;
    int16_t  lastWritten = -1;
} g_sv[2];

bool     g_led = false;
bool     g_buzz = false;
uint32_t g_beepUntil = 0;
float    g_slewDps = SERVO_DEG_PER_SEC;

uint8_t expectedPrescale(uint32_t osc) {
    // Same formula as Adafruit_PWMServoDriver::setPWMFreq().
    float v = ((static_cast<float>(osc) / (50.0f * 4096.0f)) + 0.5f) - 1.0f;
    if (v < 3) v = 3;
    if (v > 255) v = 255;
    return static_cast<uint8_t>(v);
}

bool pcaDigital(uint8_t ch, bool on) {
    if (!g_pca.up) return false;
    const uint8_t rc = on ? pca.setPWM(ch, PCA_FULL_BIT, 0) : pca.setPWM(ch, 0, PCA_FULL_BIT);
    if (rc != 0) ++g_pca.writeFail;
    return rc == 0;
}

uint16_t angleTicks(int a) {
    return static_cast<uint16_t>(map(static_cast<long>(a), 0L, 180L, SERVO_MIN_TICKS, SERVO_MAX_TICKS));
}

void servoWrite(ServoState& s) {
    if (!g_pca.up) return;
    const int a = static_cast<int>(lroundf(s.cur));
    s.ticks = angleTicks(a);
    if (pca.setPWM(s.ch, 0, s.ticks) != 0) ++g_pca.writeFail;
    s.lastWritten = static_cast<int16_t>(a);
}

void servoOff(ServoState& s) {
    s.en = false;
    s.lastWritten = -1;
    pcaDigital(s.ch, false);   // no pulse: servo goes limp
}

/** Re-send everything the host has turned on (after a PCA brown-out). */
void pcaRestore() {
    pcaDigital(PcaChannels::STATUS_LED, g_led);
    pcaDigital(PcaChannels::BUZZER, g_buzz);
    for (uint8_t i = 0; i < 2; ++i) if (g_sv[i].en) servoWrite(g_sv[i]);
}

bool pcaInit() {
    PcaState& p = g_pca;
    p.ack = i2cProbe(Hw::PCA9685_ADDR);
    p.up = false;
    if (!p.ack) return false;
    // Kill every output BEFORE begin(): begin() runs the chip at 1 kHz for a
    // moment, and after a warm reset the old servo pulse would be replayed
    // at that rate.
    i2cWriteReg(Hw::PCA9685_ADDR, PCA_REG_ALL_OFF_H, 0x10);
    pca.begin();
    pca.setOscillatorFrequency(p.osc);
    pca.setPWMFreq(50);
    i2cWriteReg(Hw::PCA9685_ADDR, PCA_REG_ALL_OFF_H, 0x10);
    p.expected = expectedPrescale(p.osc);
    const bool rd = i2cReadReg(Hw::PCA9685_ADDR, PCA_REG_MODE1, p.mode1) &&
                    i2cReadReg(Hw::PCA9685_ADDR, PCA_REG_PRESCALE, p.prescale);
    p.up = rd && (p.mode1 & PCA_MODE1_SLEEP) == 0 && p.prescale != 0;
    ++p.inits;
    p.lastCheckMs = millis();
    // Individual channel registers still hold old values; write full-OFF so
    // clearing ALL_OFF later cannot resurrect them.
    if (p.up) for (uint8_t ch = 0; ch < 16; ++ch) pcaDigital(ch, false);
    return p.up;
}

void pcaService(uint32_t now) {
    PcaState& p = g_pca;
    if (now - p.lastCheckMs < Hw::PCA_HEALTH_PERIOD_MS) return;
    p.lastCheckMs = now;
    uint8_t m1 = 0, pre = 0;
    const bool rd = i2cReadReg(Hw::PCA9685_ADDR, PCA_REG_MODE1, m1) &&
                    i2cReadReg(Hw::PCA9685_ADDR, PCA_REG_PRESCALE, pre);
    p.ack = rd;
    if (!rd) { p.up = false; return; }
    p.mode1 = m1;
    p.prescale = pre;
    if ((m1 & PCA_MODE1_SLEEP) != 0 || pre != p.expected || !p.up) {
        // It reset (servo-spike brown-out) or came back: re-init and re-send.
        ++p.resets;
        if (pcaInit()) pcaRestore();
    }
}

void servosBegin() {
    g_sv[0].ch = PcaChannels::SERVO_ARM;
    g_sv[0].cfgLo = min(Hw::ARM_ANGLE_DOWN, Hw::ARM_ANGLE_CARRY);
    g_sv[0].cfgHi = max(Hw::ARM_ANGLE_DOWN, Hw::ARM_ANGLE_CARRY);
    g_sv[1].ch = PcaChannels::SERVO_GRIPPER;
    g_sv[1].cfgLo = min(Hw::GRIPPER_CLOSED, Hw::GRIPPER_OPEN);
    g_sv[1].cfgHi = max(Hw::GRIPPER_CLOSED, Hw::GRIPPER_OPEN);
    for (uint8_t i = 0; i < 2; ++i) {
        g_sv[i].lo = g_sv[i].cfgLo;
        g_sv[i].hi = g_sv[i].cfgHi;
        g_sv[i].en = false;            // never moves at boot
        g_sv[i].cur = g_sv[i].tgt = g_sv[i].cfgLo;
    }
}

void servosService(float dt) {
    for (uint8_t i = 0; i < 2; ++i) {
        ServoState& s = g_sv[i];
        if (!s.en) continue;
        const float diff = s.tgt - s.cur;
        if (fabsf(diff) > 0.01f) {
            const float step = g_slewDps * dt;
            s.cur = (fabsf(diff) <= step) ? static_cast<float>(s.tgt) : s.cur + (diff > 0 ? step : -step);
        }
        if (static_cast<int>(lroundf(s.cur)) != s.lastWritten) servoWrite(s);
    }
}

void buzzerService(uint32_t now) {
    if (g_buzz && g_beepUntil != 0 && static_cast<int32_t>(now - g_beepUntil) >= 0) {
        g_buzz = false;
        g_beepUntil = 0;
        pcaDigital(PcaChannels::BUZZER, false);
    }
}

void beep(uint16_t ms) {
    g_buzz = true;
    g_beepUntil = millis() + ms;
    if (g_beepUntil == 0) g_beepUntil = 1;
    pcaDigital(PcaChannels::BUZZER, true);
}

// ============================================================================
//  SECTION L — COLOUR (2x TCS3200, shared S2/S3), same method as ColorVision
// ============================================================================

uint8_t  g_colSel = 3;               // bit0 = left (C1), bit1 = right (C2)
uint16_t g_col[2][4] = { { 0 } };    // [head][r,g,b,c] intensity (500000/halfPeriodUs), 0 = timeout
uint8_t  g_colStep = 0;
uint32_t g_colFrames = 0;
const uint8_t kColOut[2] = { Pins::COLOR_OUT_LEFT, Pins::COLOR_OUT_RIGHT };
// Filter order as ColorVision::update(): R, B, CLEAR, G
const bool    kS2[4]   = { false, false, true,  true };
const bool    kS3[4]   = { false, true,  false, true };
const uint8_t kSlot[4] = { 0, 2, 3, 1 };   // -> index in [r,g,b,c]

void colorBegin() {
    pinMode(Pins::COLOR_S2, OUTPUT);
    pinMode(Pins::COLOR_S3, OUTPUT);
    pinMode(Pins::COLOR_OUT_LEFT, INPUT);
    pinMode(Pins::COLOR_OUT_RIGHT, INPUT);
    digitalWrite(Pins::COLOR_S2, LOW);
    digitalWrite(Pins::COLOR_S3, LOW);
}

void colorService() {
    if (g_colSel == 0) return;
    // One filter per loop pass keeps the worst-case block at ~6 ms.
    digitalWrite(Pins::COLOR_S2, kS2[g_colStep] ? HIGH : LOW);
    digitalWrite(Pins::COLOR_S3, kS3[g_colStep] ? HIGH : LOW);
    delayMicroseconds(COLOR_SETTLE_US);
    for (uint8_t h = 0; h < 2; ++h) {
        if (!(g_colSel & (1u << h))) continue;
        const uint32_t half = pulseIn(kColOut[h], LOW, COLOR_PULSE_TIMEOUT_US);
        uint32_t v = half ? 500000UL / half : 0;
        if (v > 65535UL) v = 65535UL;
        g_col[h][kSlot[g_colStep]] = static_cast<uint16_t>(v);
    }
    if (++g_colStep >= 4) { g_colStep = 0; ++g_colFrames; }
}

// ============================================================================
//  SECTION M — START BUTTON + optional self-test
// ============================================================================

bool     g_btnStable = false;   // true = pressed
bool     g_btnLastRaw = false;
uint32_t g_btnChangeMs = 0;
uint32_t g_btnPresses = 0;

enum class SelfTest : uint8_t { IDLE, COUNTDOWN, RUN, DONE };
SelfTest g_st = SelfTest::IDLE;
uint32_t g_stT0 = 0;
uint8_t  g_stStep = 0;
uint8_t  g_stBeeps = 0;

struct StStep { int16_t l, r; uint16_t ms; };
const StStep kSelfSteps[] = {
    {  SELFTEST_PWM, 0, SELFTEST_STEP_MS }, { 0, 0, SELFTEST_GAP_MS },
    { -SELFTEST_PWM, 0, SELFTEST_STEP_MS }, { 0, 0, SELFTEST_GAP_MS },
    { 0,  SELFTEST_PWM, SELFTEST_STEP_MS }, { 0, 0, SELFTEST_GAP_MS },
    { 0, -SELFTEST_PWM, SELFTEST_STEP_MS }, { 0, 0, SELFTEST_GAP_MS },
};
constexpr uint8_t kSelfStepCount = sizeof(kSelfSteps) / sizeof(kSelfSteps[0]);

void selfTestAbort() {
    if (g_st == SelfTest::COUNTDOWN || g_st == SelfTest::RUN) {
        motorsOff();
        g_st = SelfTest::DONE;
        emit("{\"ev\":\"selftest\",\"state\":\"aborted\"}");
    }
}

void selfTestService(uint32_t now) {
#if BENCH_SELFTEST_ENABLED
    if (g_st == SelfTest::COUNTDOWN) {
        const uint32_t el = now - g_stT0;
        if (el / 1000 >= g_stBeeps && g_stBeeps < SELFTEST_COUNTDOWN_MS / 1000) {
            beep(SELFTEST_BEEP_MS);
            ++g_stBeeps;
        }
        if (el >= SELFTEST_COUNTDOWN_MS) {
            g_st = SelfTest::RUN;
            g_stStep = 0;
            g_stT0 = now;
            setMotorsFw(kSelfSteps[0].l, kSelfSteps[0].r);
        }
    } else if (g_st == SelfTest::RUN) {
        if (now - g_stT0 >= kSelfSteps[g_stStep].ms) {
            if (++g_stStep >= kSelfStepCount) {
                motorsOff();
                g_st = SelfTest::DONE;
                beep(400);
                emit("{\"ev\":\"selftest\",\"state\":\"done\"}");
                return;
            }
            g_stT0 = now;
            setMotorsFw(kSelfSteps[g_stStep].l, kSelfSteps[g_stStep].r);
        }
    }
#else
    (void)now;
#endif
}

void buttonService(uint32_t now) {
    const bool raw = digitalRead(Pins::START_BUTTON) == LOW;   // active LOW
    if (raw != g_btnLastRaw) { g_btnLastRaw = raw; g_btnChangeMs = now; }
    if (raw != g_btnStable && (now - g_btnChangeMs) >= 30) {
        g_btnStable = raw;
        if (raw) {
            ++g_btnPresses;
#if BENCH_SELFTEST_ENABLED
            if (g_st == SelfTest::COUNTDOWN || g_st == SelfTest::RUN) {
                selfTestAbort();
            } else if (!g_hostSeen && g_st == SelfTest::IDLE) {
                g_st = SelfTest::COUNTDOWN;
                g_stT0 = now;
                g_stBeeps = 0;
                emit("{\"ev\":\"selftest\",\"state\":\"countdown\"}");
            }
#endif
        }
    }
}

// ============================================================================
//  SECTION N — DEVICE-RUN SEQUENCES (direction test, deadband finder)
// ============================================================================

enum class Seq : uint8_t { NONE, DIR_PULSE, DIR_SETTLE, DB };
Seq      g_seq = Seq::NONE;
uint32_t g_seqT0 = 0;
uint8_t  g_seqSide = 0;      // DIR: 0/1. DB: 0 = L, 1 = R, 2 = pivot
int8_t   g_seqDir = 1;
int16_t  g_seqPwm = 0;
int32_t  g_seqEnc0Raw[2] = { 0, 0 };
int16_t  g_dbMove[2] = { -1, -1 };

const char* seqName() {
    switch (g_seq) {
        case Seq::DIR_PULSE:
        case Seq::DIR_SETTLE: return "dir";
        case Seq::DB:         return "db";
        default:              return (g_st == SelfTest::RUN || g_st == SelfTest::COUNTDOWN) ? "selftest" : "none";
    }
}

void seqCancel(const char* why) {
    if (g_seq == Seq::NONE) return;
    g_seq = Seq::NONE;
    motorsOff();
    emit("{\"ev\":\"seq\",\"state\":\"aborted\",\"why\":\"%s\"}", why);
}

void dbApply() {
    const int16_t p = static_cast<int16_t>(g_seqPwm * g_seqDir);
    if (g_seqSide == 0)      setMotorsFw(p, 0);
    else if (g_seqSide == 1) setMotorsFw(0, p);
    else                     setMotorsFw(static_cast<int16_t>(-p), p);   // pivot LEFT (CCW)
}

void dbFinish(const char* result) {
    g_seq = Seq::NONE;
    motorsOff();
    emit("{\"ev\":\"db\",\"mode\":\"%c\",\"dir\":%d,\"move\":[%d,%d],\"cap\":%d,\"step\":%d,\"result\":\"%s\"}",
         g_seqSide == 0 ? 'L' : (g_seqSide == 1 ? 'R' : 'P'), g_seqDir,
         g_dbMove[0], g_dbMove[1], g_capPwm, DB_STEP_PWM, result);
}

void seqService(uint32_t now) {
    if (g_seq == Seq::NONE) return;
    if (!hostAlive(now)) { seqCancel("no heartbeat"); return; }

    if (g_seq == Seq::DIR_PULSE) {
        if (now - g_seqT0 >= DIR_PULSE_MS) {
            motorsOff();
            g_seq = Seq::DIR_SETTLE;
            g_seqT0 = now;
        }
    } else if (g_seq == Seq::DIR_SETTLE) {
        if (now - g_seqT0 >= DIR_SETTLE_MS) {
            g_seq = Seq::NONE;
            const int32_t dRaw[2] = { encRaw(0) - g_seqEnc0Raw[0], encRaw(1) - g_seqEnc0Raw[1] };
            const int32_t dFw[2]  = { FW_INVERT_ENC[0] ? -dRaw[0] : dRaw[0],
                                      FW_INVERT_ENC[1] ? -dRaw[1] : dRaw[1] };
            emit("{\"ev\":\"dir\",\"side\":\"%c\",\"pwm\":%d,\"d_raw\":[%ld,%ld],\"d_fw\":[%ld,%ld],"
                 "\"inv_mot\":[%d,%d],\"inv_enc\":[%d,%d]}",
                 g_seqSide ? 'R' : 'L', g_seqPwm,
                 static_cast<long>(dRaw[0]), static_cast<long>(dRaw[1]),
                 static_cast<long>(dFw[0]), static_cast<long>(dFw[1]),
                 FW_INVERT_MOTOR[0], FW_INVERT_MOTOR[1], FW_INVERT_ENC[0], FW_INVERT_ENC[1]);
        }
    } else if (g_seq == Seq::DB) {
        // Did a tested side move since the start?
        for (uint8_t s = 0; s < 2; ++s) {
            const bool tested = (g_seqSide == 2) || (g_seqSide == s);
            if (!tested || g_dbMove[s] >= 0) continue;
            const int32_t d = encRaw(s) - g_seqEnc0Raw[s];
            if ((d < 0 ? -d : d) >= DB_MOVE_COUNTS) g_dbMove[s] = g_seqPwm;
        }
        const bool done = (g_seqSide == 2) ? (g_dbMove[0] >= 0 && g_dbMove[1] >= 0)
                                           : (g_dbMove[g_seqSide] >= 0);
        if (done) { dbFinish("moved"); return; }
        if (now - g_seqT0 >= DB_STEP_MS) {
            g_seqT0 = now;
            g_seqPwm = static_cast<int16_t>(g_seqPwm + DB_STEP_PWM);
            if (g_seqPwm > g_capPwm) { dbFinish("cap"); return; }
            dbApply();
        }
    }
}

// ============================================================================
//  SECTION O — STOP + HELLO
// ============================================================================

void stopAll(const char* why) {
    motorsOff();
    if (g_seq != Seq::NONE) g_seq = Seq::NONE;
    selfTestAbort();
    for (uint8_t i = 0; i < 2; ++i) servoOff(g_sv[i]);
    g_buzz = false;
    g_beepUntil = 0;
    pcaDigital(PcaChannels::BUZZER, false);
    g_led = false;
    pcaDigital(PcaChannels::STATUS_LED, false);
    emit("{\"ev\":\"stop\",\"why\":\"%s\"}", why);
}

void emitHello() {
    Line& L = g_line;
    L.add("{\"ev\":\"hello\",\"fw\":\"syrobix-bench\",\"ver\":1,\"cfg\":{");
    L.add("\"PWM_MAX\":%d,\"PWM_MIN_MOVE\":%d,\"PWM_FREQ_HZ\":%lu,\"cap_default_pct\":%u,",
          Hw::PWM_MAX, Hw::PWM_MIN_MOVE, static_cast<unsigned long>(Hw::PWM_FREQ_HZ), PWM_CAP_DEFAULT_PCT);
    L.add("\"SPEED_APPROACH\":%d,\"SPEED_PIVOT\":%d,", Tune::SPEED_APPROACH, Tune::SPEED_PIVOT);
    L.add("\"motor_deadman_ms\":%lu,\"host_deadman_ms\":%lu,\"dir_pwm\":%d,\"db_step\":%d,\"db_counts\":%ld,",
          static_cast<unsigned long>(MOTOR_DEADMAN_MS), static_cast<unsigned long>(HOST_DEADMAN_MS),
          DIR_TEST_PWM_DEFAULT, DB_STEP_PWM, static_cast<long>(DB_MOVE_COUNTS));
    L.add("\"inv_mot\":[%d,%d],\"inv_enc\":[%d,%d],",
          FW_INVERT_MOTOR[0], FW_INVERT_MOTOR[1], FW_INVERT_ENC[0], FW_INVERT_ENC[1]);
    L.add("\"WHEEL_DIAMETER_MM\":%.2f,\"TRACK_WIDTH_MM\":%.2f,\"ENCODER_CPR\":%.1f,\"MM_PER_COUNT\":%.5f,",
          Hw::WHEEL_DIAMETER_MM, Hw::TRACK_WIDTH_MM, Hw::ENCODER_CPR, Hw::MM_PER_COUNT);
    L.add("\"LINE_ARRAY_TO_PIVOT_MM\":%.1f,\"COLOR_HEADS_TO_CENTER_MM\":%.1f,\"COLOR_HEADS_BEHIND_ARRAY_MM\":%.1f,",
          Hw::LINE_ARRAY_TO_PIVOT_MM, Hw::COLOR_HEADS_TO_CENTER_MM, Hw::COLOR_HEADS_BEHIND_ARRAY_MM);
    L.add("\"IMU_YAW_SIGN\":%.0f,\"MPU6500_ADDR\":%u,\"gyro_deadband\":%.2f,",
          Hw::IMU_YAW_SIGN, Hw::MPU6500_ADDR, GYRO_DEADBAND_DPS);
    L.add("\"PCA9685_ADDR\":%u,\"PCA_OSC_HZ\":%lu,\"SERVO_MIN_US\":%u,\"SERVO_MAX_US\":%u,\"SERVO_MAX_ANGLE\":%u,",
          Hw::PCA9685_ADDR, static_cast<unsigned long>(Hw::PCA_OSC_HZ), Hw::SERVO_MIN_US, Hw::SERVO_MAX_US,
          Hw::SERVO_MAX_ANGLE);
    L.add("\"servo_ticks\":[%ld,%ld],\"slew_dps\":%.1f,",
          SERVO_MIN_TICKS, SERVO_MAX_TICKS, SERVO_DEG_PER_SEC);
    L.add("\"ARM_ANGLE_DOWN\":%u,\"ARM_ANGLE_CARRY\":%u,\"GRIPPER_OPEN\":%u,\"GRIPPER_CLOSED\":%u,",
          Hw::ARM_ANGLE_DOWN, Hw::ARM_ANGLE_CARRY, Hw::GRIPPER_OPEN, Hw::GRIPPER_CLOSED);
    L.add("\"LINE_ON_THRESHOLD\":%u,\"MIN_USABLE_SPREAD\":%u,\"INVERT_POLARITY\":%d,",
          Tune::LINE_ON_THRESHOLD, LINE_MIN_USABLE_SPREAD, LINE_INVERT_POLARITY);
    L.add("\"TOF_ADDR\":[%u,%u,%u],\"TOF_OFFSET_MM\":[%d,%d,%d],\"TOF_MIN_RELIABLE_MM\":%u,\"xshut_open_drain\":%d,",
          Hw::TOF_ADDR_FRONT, Hw::TOF_ADDR_LEFT, Hw::TOF_ADDR_RIGHT,
          Tof::OFFSET_MM[0], Tof::OFFSET_MM[1], Tof::OFFSET_MM[2], Tof::MIN_RELIABLE_MM,
          SYROBIX_TOF_XSHUT_OPEN_DRAIN);
    L.add("\"selftest\":%d},", BENCH_SELFTEST_ENABLED);
    L.add("\"boot\":{\"imu_ack\":%d,\"who\":%u,\"imu_cfg\":%d,\"tof_fail\":[%u,%u,%u],\"tof_stuck\":%u,"
          "\"pca_ack\":%d,\"pca_up\":%d,\"i2c\":[",
          g_imu.ack, g_imu.who, g_imu.cfgOk, g_tofSt[0].fail, g_tofSt[1].fail, g_tofSt[2].fail,
          g_tofStuckMask, g_pca.ack, g_pca.up);
    for (uint8_t i = 0; i < g_scanN; ++i) L.add("%s%u", i ? "," : "", g_scan[i]);
    L.add("]}}");
    L.send();
}

// ============================================================================
//  SECTION P — COMMANDS
// ============================================================================

char     g_rx[160];
uint8_t  g_rxN = 0;
bool     g_rxOverflow = false;

bool parseInt(const char* s, long& out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    out = strtol(s, &end, 10);
    return end && *end == '\0';
}

int servoIndex(const char* s) {
    if (!s) return -1;
    if (!strcmp(s, "1") || !strcmp(s, "S1")) return 0;
    if (!strcmp(s, "2") || !strcmp(s, "S2")) return 1;
    return -1;
}

void handleCommand(char* line) {
    const uint32_t now = millis();
    g_lastHostMs = now;
    if (!g_hostSeen) { g_hostSeen = true; selfTestAbort(); }

    char* tok[8] = { nullptr };
    uint8_t nt = 0;
    for (char* p = strtok(line, " \t"); p && nt < 8; p = strtok(nullptr, " \t")) {
        for (char* q = p; *q; ++q) *q = static_cast<char>(toupper(*q));
        tok[nt++] = p;
    }
    if (nt == 0) return;
    const char* c = tok[0];
    long a = 0, b = 0;

    if (!strcmp(c, "HB")) return;
    if (!strcmp(c, "STOP")) { stopAll("cmd"); return; }
    if (!strcmp(c, "HELLO")) { emitHello(); return; }

    if (!strcmp(c, "M")) {
        if (nt < 3 || !parseInt(tok[1], a) || !parseInt(tok[2], b)) { emitErr(c, "usage: M <left> <right>"); return; }
        if (g_seq != Seq::NONE) seqCancel("manual");
        setMotorsFw(static_cast<int16_t>(constrain(a, -Hw::PWM_MAX, Hw::PWM_MAX)),
                    static_cast<int16_t>(constrain(b, -Hw::PWM_MAX, Hw::PWM_MAX)));
        g_motorCmdMs = now;
        return;   // no ack: sent every 150 ms
    }
    if (!strcmp(c, "CAP")) {
        if (nt < 2 || !parseInt(tok[1], a) || a < 5 || a > 100) { emitErr(c, "usage: CAP <5..100> [YES]"); return; }
        if (a > PWM_CAP_DEFAULT_PCT && !(nt >= 3 && !strcmp(tok[2], "YES"))) {
            emitErr(c, "above 40 percent needs YES"); return;
        }
        g_capPct = static_cast<uint8_t>(a);
        g_capPwm = static_cast<int16_t>((Hw::PWM_MAX * a) / 100);
        setMotorsFw(g_motFw[0], g_motFw[1]);   // re-clamp anything running
        emit("{\"ev\":\"cap\",\"pct\":%u,\"pwm\":%d}", g_capPct, g_capPwm);
        return;
    }
    if (!strcmp(c, "ENC")) { encReset(); emitAck("ENC"); return; }
    if (!strcmp(c, "DIR")) {
        if (nt < 2 || (strcmp(tok[1], "L") && strcmp(tok[1], "R"))) { emitErr(c, "usage: DIR <L|R> [pwm]"); return; }
        long p = DIR_TEST_PWM_DEFAULT;
        if (nt >= 3 && (!parseInt(tok[2], p) || p <= 0)) { emitErr(c, "bad pwm"); return; }
        if (p > g_capPwm) p = g_capPwm;
        g_seqSide = (tok[1][0] == 'R') ? 1 : 0;
        g_seqPwm = static_cast<int16_t>(p);
        g_seqEnc0Raw[0] = encRaw(0);
        g_seqEnc0Raw[1] = encRaw(1);
        g_seq = Seq::DIR_PULSE;
        g_seqT0 = now;
        if (g_seqSide == 0) setMotorsFw(g_seqPwm, 0); else setMotorsFw(0, g_seqPwm);
        emitAck("DIR");
        return;
    }
    if (!strcmp(c, "DB")) {
        if (nt < 2 || (strcmp(tok[1], "L") && strcmp(tok[1], "R") && strcmp(tok[1], "P"))) {
            emitErr(c, "usage: DB <L|R|P> [+|-]"); return;
        }
        g_seqSide = (tok[1][0] == 'L') ? 0 : (tok[1][0] == 'R') ? 1 : 2;
        g_seqDir = (nt >= 3 && tok[2][0] == '-') ? -1 : 1;
        g_seqPwm = 0;
        g_dbMove[0] = g_dbMove[1] = -1;
        g_seqEnc0Raw[0] = encRaw(0);
        g_seqEnc0Raw[1] = encRaw(1);
        g_seq = Seq::DB;
        g_seqT0 = now;
        emitAck("DB");
        return;
    }
    if (!strcmp(c, "IMU")) {
        if (nt >= 2 && !strcmp(tok[1], "INIT")) {
            imuBegin();
            emit("{\"ev\":\"imu_init\",\"ack\":%d,\"who\":%u,\"cfg\":%d}", g_imu.ack, g_imu.who, g_imu.cfgOk);
            return;
        }
        if (nt >= 2 && !strcmp(tok[1], "STILL")) {
            long ms = 3000;
            if (nt >= 3 && (!parseInt(tok[2], ms) || ms < 500 || ms > 10000)) { emitErr(c, "ms 500..10000"); return; }
            if (!g_imu.present) { emitErr(c, "imu not present"); return; }
            imuStillStart(static_cast<uint32_t>(ms));
            emitAck("IMU STILL");
            return;
        }
        if (nt >= 2 && !strcmp(tok[1], "ZERO")) { g_imu.yaw = 0; emitAck("IMU ZERO"); return; }
        emitErr(c, "usage: IMU INIT|STILL [ms]|ZERO");
        return;
    }
    if (!strcmp(c, "TOF")) {
        if (nt >= 2 && !strcmp(tok[1], "INIT")) {
            tofBegin();
            emit("{\"ev\":\"tof_init\",\"fail\":[%u,%u,%u],\"stuck\":%u}",
                 g_tofSt[0].fail, g_tofSt[1].fail, g_tofSt[2].fail, g_tofStuckMask);
            return;
        }
        if (nt >= 3 && !strcmp(tok[1], "STAT")) {
            if (!parseInt(tok[2], a) || a < 1 || a > 3) { emitErr(c, "id 1..3"); return; }
            long ms = 2000;
            if (nt >= 4 && (!parseInt(tok[3], ms) || ms < 200 || ms > 10000)) { emitErr(c, "ms 200..10000"); return; }
            TofState& s = g_tofSt[a - 1];
            if (!s.up) { emitErr(c, "sensor not up"); return; }
            s.statRun = true; s.statStart = now; s.statEnd = now + ms;
            s.sN = s.sBad = 0; s.sSum = s.sSq = 0; s.sMin = 0xFFFF; s.sMax = 0;
            emitAck("TOF STAT");
            return;
        }
        if (nt >= 5 && !strcmp(tok[1], "CFG")) {
            long bud = 0;
            if (!parseInt(tok[2], a) || a < 1 || a > 3 || !parseInt(tok[4], bud)) { emitErr(c, "usage: TOF CFG <id> <S|M|L> <budget_ms>"); return; }
            TofDistanceMode m = tok[3][0] == 'S' ? TofDistanceMode::SHORT
                              : tok[3][0] == 'M' ? TofDistanceMode::MEDIUM : TofDistanceMode::LONG;
            if (!g_tofSt[a - 1].up) { emitErr(c, "sensor not up"); return; }
            if (bud < minBudget(m) || bud > Tof::MAX_BUDGET_MS) { emitErr(c, "budget below mode minimum"); return; }
            const bool ok = tofApply(static_cast<uint8_t>(a - 1), m, static_cast<uint16_t>(bud),
                                     static_cast<uint16_t>(bud + Tof::PERIOD_SLACK_MS));
            if (!ok) { emitErr(c, "rejected by sensor"); return; }
            emitAck("TOF CFG");
            return;
        }
        emitErr(c, "usage: TOF INIT|STAT <id> [ms]|CFG <id> <S|M|L> <ms>");
        return;
    }
    if (!strcmp(c, "I2C")) { i2cScan(); emitScan(); return; }
    if (!strcmp(c, "PCA")) {
        if (nt >= 2 && !strcmp(tok[1], "INIT")) { if (pcaInit()) pcaRestore(); }
        else if (nt >= 3 && !strcmp(tok[1], "OSC")) {
            if (!parseInt(tok[2], a) || a < 20000000L || a > 30000000L) { emitErr(c, "osc 20000000..30000000"); return; }
            g_pca.osc = static_cast<uint32_t>(a);
            if (pcaInit()) pcaRestore();
        } else if (!(nt >= 2 && !strcmp(tok[1], "STAT"))) { emitErr(c, "usage: PCA INIT|STAT|OSC <hz>"); return; }
        emit("{\"ev\":\"pca\",\"ack\":%d,\"up\":%d,\"mode1\":%u,\"prescale\":%u,\"expected\":%u,\"osc\":%lu,\"inits\":%lu}",
             g_pca.ack, g_pca.up, g_pca.mode1, g_pca.prescale, g_pca.expected,
             static_cast<unsigned long>(g_pca.osc), static_cast<unsigned long>(g_pca.inits));
        return;
    }
    if (!strcmp(c, "SV")) {
        const int i = servoIndex(nt >= 2 ? tok[1] : nullptr);
        if (i < 0 || nt < 3) { emitErr(c, "usage: SV <1|2> ON <deg> | OFF | <deg>"); return; }
        ServoState& s = g_sv[i];
        if (!g_pca.up) { emitErr(c, "PCA9685 not up"); return; }
        if (!strcmp(tok[2], "OFF")) { servoOff(s); emitAck("SV OFF"); return; }
        if (!strcmp(tok[2], "ON")) {
            if (nt < 4 || !parseInt(tok[3], a)) { emitErr(c, "usage: SV <n> ON <deg>"); return; }
            s.cur = s.tgt = static_cast<int16_t>(constrain(a, s.lo, s.hi));
            s.en = true;
            servoWrite(s);   // first pulse: jumps to this angle (real position unknown)
            emitAck("SV ON");
            return;
        }
        if (!parseInt(tok[2], a)) { emitErr(c, "bad angle"); return; }
        if (!s.en) { emitErr(c, "enable first: SV <n> ON <deg>"); return; }
        s.tgt = static_cast<int16_t>(constrain(a, s.lo, s.hi));
        return;   // no ack: sliders send often
    }
    if (!strcmp(c, "SLIM")) {
        const int i = servoIndex(nt >= 2 ? tok[1] : nullptr);
        long lo = 0, hi = 0;
        if (i < 0 || nt < 4 || !parseInt(tok[2], lo) || !parseInt(tok[3], hi) || lo > hi) {
            emitErr(c, "usage: SLIM <1|2> <lo> <hi> [YES]"); return;
        }
        ServoState& s = g_sv[i];
        if (lo < 0) lo = 0;
        if (hi > Hw::SERVO_MAX_ANGLE) hi = Hw::SERVO_MAX_ANGLE;
        if ((lo < s.cfgLo || hi > s.cfgHi) && !(nt >= 5 && !strcmp(tok[4], "YES"))) {
            emitErr(c, "outside Config.h pose range needs YES"); return;
        }
        s.lo = static_cast<uint8_t>(lo);
        s.hi = static_cast<uint8_t>(hi);
        s.tgt = static_cast<int16_t>(constrain(s.tgt, s.lo, s.hi));
        emitAck("SLIM");
        return;
    }
    if (!strcmp(c, "SLEW")) {
        if (nt < 2 || !parseInt(tok[1], a) || a < 10 || a > 360) { emitErr(c, "usage: SLEW <10..360>"); return; }
        g_slewDps = static_cast<float>(a);
        emitAck("SLEW");
        return;
    }
    if (!strcmp(c, "OUT")) {
        if (nt < 3 || !parseInt(tok[2], a)) { emitErr(c, "usage: OUT <D1|B1> <0|1>"); return; }
        if (!strcmp(tok[1], "D1")) { g_led = a != 0; pcaDigital(PcaChannels::STATUS_LED, g_led); }
        else if (!strcmp(tok[1], "B1")) {
            g_buzz = a != 0; g_beepUntil = g_buzz ? millis() + BEEP_MAX_MS : 0;   // never stuck on
            pcaDigital(PcaChannels::BUZZER, g_buzz);
        } else { emitErr(c, "D1 or B1"); return; }
        emitAck("OUT");
        return;
    }
    if (!strcmp(c, "BEEP")) {
        if (nt < 2 || !parseInt(tok[1], a) || a < 1 || a > BEEP_MAX_MS) { emitErr(c, "usage: BEEP <1..3000>"); return; }
        beep(static_cast<uint16_t>(a));
        emitAck("BEEP");
        return;
    }
    if (!strcmp(c, "COL")) {
        if (nt < 2) { emitErr(c, "usage: COL <L|R|B|OFF>"); return; }
        if (!strcmp(tok[1], "L")) g_colSel = 1;
        else if (!strcmp(tok[1], "R")) g_colSel = 2;
        else if (!strcmp(tok[1], "B")) g_colSel = 3;
        else if (!strcmp(tok[1], "OFF")) g_colSel = 0;
        else { emitErr(c, "L, R, B or OFF"); return; }
        emitAck("COL");
        return;
    }
    if (!strcmp(c, "TELE")) {
        if (nt < 2 || !parseInt(tok[1], a) || a < 20 || a > 1000) { emitErr(c, "usage: TELE <20..1000>"); return; }
        g_teleMs = static_cast<uint32_t>(a);
        emitAck("TELE");
        return;
    }
    emitErr(c, "unknown command");
}

void serialService() {
    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch < 0) break;
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (!g_rxOverflow && g_rxN > 0) {
                g_rx[g_rxN] = '\0';
                handleCommand(g_rx);
            } else if (g_rxOverflow) {
                emitErr("?", "line too long");
            }
            g_rxN = 0;
            g_rxOverflow = false;
            continue;
        }
        if (g_rxN < sizeof(g_rx) - 1) g_rx[g_rxN++] = static_cast<char>(ch);
        else g_rxOverflow = true;
    }
}

// ============================================================================
//  SECTION Q — TELEMETRY
// ============================================================================

uint32_t g_teleLast = 0;
uint32_t g_loopMaxUs = 0;

void telemetry(uint32_t now) {
    if (now - g_teleLast < g_teleMs) return;
    g_teleLast = now;
    Line& L = g_line;
    uint32_t bad0, bad1;
    portENTER_CRITICAL(&g_encMux);
    bad0 = g_encBad[0]; bad1 = g_encBad[1];
    portEXIT_CRITICAL(&g_encMux);

    L.add("{\"ty\":\"tl\",\"t\":%lu,\"loop\":%lu,", static_cast<unsigned long>(now),
          static_cast<unsigned long>(g_loopMaxUs));
    g_loopMaxUs = 0;
    L.add("\"m\":{\"fw\":[%d,%d],\"raw\":[%d,%d],\"stby\":%d,\"cap\":%d,\"pct\":%u,\"seq\":\"%s\"},",
          g_motFw[0], g_motFw[1], g_motRaw[0], g_motRaw[1], g_stby, g_capPwm, g_capPct, seqName());
    L.add("\"e\":{\"raw\":[%ld,%ld],\"fw\":[%ld,%ld],\"bad\":[%lu,%lu]},",
          static_cast<long>(encRaw(0)), static_cast<long>(encRaw(1)),
          static_cast<long>(encFw(0)), static_cast<long>(encFw(1)),
          static_cast<unsigned long>(bad0), static_cast<unsigned long>(bad1));
    L.add("\"l\":[%u,%u,%u,%u,%u,%u,%u,%u],",
          g_lineRaw[0], g_lineRaw[1], g_lineRaw[2], g_lineRaw[3],
          g_lineRaw[4], g_lineRaw[5], g_lineRaw[6], g_lineRaw[7]);
    const ImuState& m = g_imu;
    L.add("\"g\":{\"ok\":%d,\"who\":%u,\"cfg\":%d,\"g\":[%.3f,%.3f,%.3f],\"a\":[%.4f,%.4f,%.4f],"
          "\"yaw\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,\"tC\":%.1f,\"bias\":%d,\"still\":%d,"
          "\"frz\":%d,\"zero\":%d,\"ff\":%d,\"err\":%lu},",
          m.present, m.who, m.cfgOk, fin(m.g[0]), fin(m.g[1]), fin(m.g[2]),
          fin(m.a[0]), fin(m.a[1]), fin(m.a[2]), fin(static_cast<float>(m.yaw)),
          fin(m.roll), fin(m.pitch), fin(m.tempC), m.biasSet, m.stillRun,
          m.frozen, m.zeros, m.ones, static_cast<unsigned long>(m.readErr));
    L.add("\"tof\":[");
    for (uint8_t i = 0; i < 3; ++i) {
        const TofState& s = g_tofSt[i];
        L.add("%s{\"up\":%d,\"f\":%u,\"mm\":%u,\"st\":%u,\"sig\":%.2f,\"amb\":%.2f,\"age\":%lu,\"n\":%lu,"
              "\"err\":%lu,\"mode\":%u,\"bud\":%u,\"stat\":%d}",
              i ? "," : "", s.up, s.fail, s.mm, s.st, fin(s.sig), fin(s.amb),
              static_cast<unsigned long>(s.stamp ? now - s.stamp : 0),
              static_cast<unsigned long>(s.n), static_cast<unsigned long>(s.err),
              static_cast<unsigned>(s.mode), s.budget, s.statRun);
    }
    L.add("],");
    const PcaState& p = g_pca;
    L.add("\"p\":{\"ack\":%d,\"up\":%d,\"m1\":%u,\"pre\":%u,\"exp\":%u,\"osc\":%lu,\"init\":%lu,\"rst\":%lu,\"wf\":%lu},",
          p.ack, p.up, p.mode1, p.prescale, p.expected, static_cast<unsigned long>(p.osc),
          static_cast<unsigned long>(p.inits), static_cast<unsigned long>(p.resets),
          static_cast<unsigned long>(p.writeFail));
    L.add("\"s\":[");
    for (uint8_t i = 0; i < 2; ++i) {
        const ServoState& s = g_sv[i];
        L.add("%s{\"en\":%d,\"cur\":%.1f,\"tgt\":%d,\"lo\":%u,\"hi\":%u,\"tk\":%u}",
              i ? "," : "", s.en, s.cur, s.tgt, s.lo, s.hi, s.ticks);
    }
    L.add("],\"slew\":%.0f,\"d1\":%d,\"b1\":%d,", g_slewDps, g_led, g_buzz);
    L.add("\"c\":{\"sel\":%u,\"f\":%lu,\"L\":[%u,%u,%u,%u],\"R\":[%u,%u,%u,%u]},",
          g_colSel, static_cast<unsigned long>(g_colFrames),
          g_col[0][0], g_col[0][1], g_col[0][2], g_col[0][3],
          g_col[1][0], g_col[1][1], g_col[1][2], g_col[1][3]);
    L.add("\"btn\":{\"p\":%d,\"n\":%lu},\"st\":%u}", g_btnStable,
          static_cast<unsigned long>(g_btnPresses), static_cast<unsigned>(g_st));
    L.send();
}

}  // namespace

// ============================================================================
//  SETUP / LOOP
// ============================================================================

void setup() {
    // Outputs safe FIRST, before anything that can take time.
    motorsBegin();
    for (uint8_t i = 0; i < 3; ++i) xshutAssert(kTof[i].xshut);
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);

    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(4096);
    Serial.begin(BENCH_BAUD);

    Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL, Hw::I2C_CLOCK_HZ);

    pcaInit();        // all PCA outputs full-OFF, servos get no pulse
    servosBegin();
    encBegin();
    lineBegin();
    colorBegin();
    imuBegin();
    tofBegin();
    i2cScan();

    emitHello();
}

void loop() {
    const uint32_t t0 = micros();
    static uint32_t lastLoopUs = t0;
    const float dt = (t0 - lastLoopUs) * 1e-6f;
    lastLoopUs = t0;
    const uint32_t now = millis();

    serialService();

    // Motor deadman: a command lives MOTOR_DEADMAN_MS unless a device-run
    // sequence (which has its own heartbeat check) or the self-test owns them.
    if (g_seq == Seq::NONE && g_st != SelfTest::RUN &&
        (g_motFw[0] != 0 || g_motFw[1] != 0) && (now - g_motorCmdMs) > MOTOR_DEADMAN_MS) {
        motorsOff();
        emit("{\"ev\":\"deadman\"}");
    }

    seqService(now);
    selfTestService(now);
    buttonService(now);
    lineService();
    imuService();
    tofService();
    pcaService(now);
    servosService(dt);
    buzzerService(now);
    colorService();
    telemetry(now);

    const uint32_t took = micros() - t0;
    if (took > g_loopMaxUs) g_loopMaxUs = took;
}
