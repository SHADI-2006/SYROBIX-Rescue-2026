/**
 * ============================================================================
 *  @file       main.cpp
 *  @project    SYROBIX Rescue Robot  |  ESP32-S3
 *  @brief      Entry point. Brings up the subsystems in dependency order, hands
 *              control to the FreeRTOS scheduler, and gets out of the way.
 *
 *  @note       Rename to main.ino for the Arduino IDE. Under PlatformIO leave
 *              it as main.cpp; the #include <Arduino.h> below is what the .ino
 *              preprocessor would otherwise have inserted for you.
 *
 *  ---------------------------------------------------------------------------
 *  TASK TOPOLOGY (created inside TaskHandler::begin(), not here)
 *  ---------------------------------------------------------------------------
 *    Core 1 — real time, never touches I2C
 *      control    prio 5, 200 Hz   PID, drivetrain, PWM output
 *      line       prio 4, 200 Hz   TCR5000 ADC sweep
 *
 *    Core 0 — latency tolerant
 *      mission    prio 3,  50 Hz   rescue state machine
 *      sensors    prio 2,  33 Hz   IMU burst, ToF round-robin, colour pair
 *      telemetry  prio 1,   5 Hz   serial diagnostics (compile-time optional)
 *
 *  You asked for two pinned tasks; the split you specified is honoured exactly.
 *  The mission machine is a third task rather than a branch inside the control
 *  loop because it must never share a deadline with the 200 Hz PID — a slow
 *  mission tick would otherwise show up as jitter in line following.
 *
 *  Cross-core traffic is deliberately tiny and one-directional per channel:
 *    sensors -> everyone   : SensorSnapshot, mutex-protected
 *    mission -> control    : line-follow flag + speed, and the packed 32-bit
 *                            drive mailbox. Only the control task writes the
 *                            motors.
 *
 *  I2C BUS (GPIO9/10) — one bus, two Core-0 users:
 *    sensors task : MPU6500 + 3x VL53L1X
 *    mission task : PCA9685 (servos, status LED, active buzzer)   [Rev 3]
 *  TwoWire's internal per-transaction mutex keeps them from interleaving.
 *  Core 1 never touches I2C.
 *
 *  There is no queue. A queue would buffer stale sensor frames, and stale is
 *  worse than dropped when the consumer is a controller.
 *
 *  ---------------------------------------------------------------------------
 *  NO delay() ANYWHERE IN THE RUNNING SYSTEM
 *  ---------------------------------------------------------------------------
 *  The only delay() calls in the codebase are inside SensorsSystem::begin(),
 *  during the VL53L1X XSHUT bring-up (LidarBank::begin()), which needs a
 *  short hard boot wait per sensor before any task exists. Every task
 *  loop paces on vTaskDelayUntil; every sequence is a phase timer.
 * ============================================================================
 */

#include <Arduino.h>

#include "Config.h"
#include "RobotDrivetrain.h"
#include "SensorsSystem.h"
#include "TaskHandler.h"


// ============================================================================
//  Singletons
// ============================================================================
//  Static storage duration, constructed before setup(). None of them touch
//  hardware in their constructors — that all happens in begin(), so the order
//  of bring-up is explicit and visible below rather than hidden in static init.
// ============================================================================

static RobotDrivetrain g_drivetrain;
static SensorsSystem   g_sensors;
static TaskHandler     g_tasks(g_drivetrain, g_sensors);


// ============================================================================
//  Fault handling
// ============================================================================

/**
 * @brief Terminal halt with a visible failure code.
 *
 * @details Called when a mandatory subsystem fails to initialise. Deliberately
 *          does NOT start the tasks: a robot that drives with a dead ranger or
 *          an unaddressed ToF bank is more dangerous to the field, and to its
 *          own score, than one that sits still and blinks.
 */
static void haltWithFault(const char* reason, uint8_t blinks) {
    // [Rev 3] The status LED is a PCA9685 channel now, so a fault that may
    // involve the PCA itself is reported on Serial (UART0 / native USB are
    // both free again). Blink count = fault code, printed as '*'.
    Serial.print(F("[FATAL] "));
    Serial.println(reason);

    pinMode(Pins::MOTOR_STBY, OUTPUT);
    digitalWrite(Pins::MOTOR_STBY, LOW);   // drivers off, hard

    for (;;) {
        for (uint8_t i = 0; i < blinks; ++i) {
            Serial.print('*');
            vTaskDelay(pdMS_TO_TICKS(180));
        }
        Serial.println();
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}


// ============================================================================
//  setup()
// ============================================================================

void setup() {
    Serial.begin(115200);
    // Bounded wait for the USB CDC to enumerate. Not delay(): if no host is
    // attached at the field, this must not stall the boot.
    const uint32_t serialDeadline = millis() + 1500;
    while (!Serial && millis() < serialDeadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println();
    Serial.println(F("=================================================="));
    Serial.println(F("  SYROBIX Rescue Robot  |  ESP32-S3"));
    Serial.printf ("  Build: %s %s\n", __DATE__, __TIME__);
    Serial.println(F("=================================================="));

    // ---- 1. Drivetrain ---------------------------------------------------
    // First, because it is what parks the motor drivers in standby. Until this
    // returns, the TB6612 STBY line is floating and the motors could twitch on
    // power-up transients.
    Serial.print(F("[boot] drivetrain ... "));
    if (!g_drivetrain.begin()) {
        haltWithFault("drivetrain init failed", 2);
    }
    Serial.println(F("ok"));

    // ---- 2. Sensors ------------------------------------------------------
    // Brings up I2C, then re-addresses the three VL53L1X units via XSHUT. This
    // must happen before anything else scans the bus: until it completes, all
    // three rangers are sitting on 0x29 together.
    Serial.print(F("[boot] sensors ... "));
    if (!g_sensors.begin()) {
        // Name the actual culprit. The old heuristic read yawRateDegS() and a
        // ToF sample before ANY update had run — both are always 0/invalid at
        // this point, so it reported "check MPU6500" for every ToF failure.
        LidarBank& tof = g_sensors.lidar();
        if (tof.busFault()) {
            haltWithFault("ToF: a device answers 0x29 with every XSHUT low "
                          "(XSHUT line not reaching a VL53L1X)", 3);
        }
        if (!tof.isUp(LidarId::FRONT)) {
            haltWithFault("FRONT VL53L1X init failed (power / XSHUT / I2C)", 3);
        }
        haltWithFault("MPU6500 not answering WHO_AM_I (I2C / wiring)", 5);
    }
    Serial.printf("ok (ToF up-mask 0x%X)\n",
                  static_cast<unsigned>(g_sensors.lidar().upMask()));

    // ---- 3. PCA9685 + tasks ---------------------------------------------
    // ORDER IS LOAD-BEARING: TaskHandler::begin() initialises the PCA9685
    // first, and Adafruit's begin() calls Wire.begin() with NO pins. Because
    // step 2 already started the bus on GPIO9/10, that call is a no-op. Run
    // this before step 2 and the PCA would claim the default I2C pins instead.
    // Then it spawns all five tasks, pinned per Rtos::.
    Serial.print(F("[boot] PCA9685 + tasks ... "));
    if (!g_tasks.begin()) {
        if (!g_tasks.peripheralsUp()) {
            haltWithFault("PCA9685 not answering at 0x40 (VCC 3.3 V? SDA/SCL? "
                          "address jumpers?)", 6);
        }
        haltWithFault("task creation failed (out of heap?)", 4);
    }
    Serial.println(F("ok"));

    Serial.println(F("[boot] calibrating — sweep the array across the line"));
    Serial.println(F("[boot] then press START to begin the run"));
}


// ============================================================================
//  loop()
// ============================================================================

/**
 * @brief Intentionally empty.
 *
 * @details Arduino's loop() runs as the lowest-priority task on Core 1. Doing
 *          real work here would contend with the 200 Hz control task for the
 *          exact core that must not be contended.
 *
 *          It is not deleted outright only because the Arduino core's own
 *          housekeeping expects the loopTask to exist. The long delay keeps it
 *          parked and off the runqueue.
 */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
