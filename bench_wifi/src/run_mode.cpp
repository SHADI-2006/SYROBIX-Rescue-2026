/**
 * ============================================================================
 *  @file       run_mode.cpp
 *  @project    SYROBIX Bench Tool (Wi-Fi edition) | RUN mode
 *  @brief      The match firmware with a Wi-Fi remote, for practice runs.
 *
 *  Boot order is copied from src/main.cpp and is load-bearing:
 *  drivetrain -> sensors (starts I2C, re-addresses ToF) -> TaskHandler (PCA,
 *  then the five tasks). Nothing in ../src is modified.
 *
 *  Remote START: the match firmware only reads the physical START button
 *  (GPIO0, active LOW, external pull-up). A virtual press pulls that pin LOW
 *  for VIRTUAL_PRESS_MS in OPEN-DRAIN mode, so the ESP32 never drives it HIGH
 *  and can never fight the real button. To the firmware it is a real press.
 *
 *  Differences from src/main.cpp (all on purpose):
 *    - A boot fault does not loop forever: STBY goes LOW and the reason is
 *      shown on the Wi-Fi page.
 *    - Optional link-loss stop: if the phone stops sending heartbeats during
 *      a run, TaskHandler::emergencyStop() is called (practice safety; a real
 *      match has no link, so this can be switched off in the UI).
 * ============================================================================
 */

#include "run_mode.h"

#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>
#include <string.h>

#include "Config.h"
#include "RobotDrivetrain.h"
#include "SensorsSystem.h"
#include "TaskHandler.h"
#include "net.h"

namespace {

constexpr uint32_t STATUS_PERIOD_MS  = 200;    // 5 Hz, like the match telemetry task
constexpr uint32_t VIRTUAL_PRESS_MS  = 200;    // > the firmware's 30 ms debounce
constexpr uint32_t LINK_LOSS_STOP_MS = 1500;   // no heartbeat this long -> STOP

RobotDrivetrain g_drivetrain;
SensorsSystem   g_sensors;
TaskHandler     g_tasks(g_drivetrain, g_sensors);

const char* g_fault       = "";
bool        g_tasksUp     = false;
uint32_t    g_pressUntil  = 0;
uint32_t    g_lastHostMs  = 0;
bool        g_linkStop    = true;
bool        g_linkStopped = false;
bool        g_userStopped = false;
uint32_t    g_lastStatus  = 0;

const char* stateName(RobotState s) {
    switch (s) {
        case RobotState::BOOT:             return "BOOT";
        case RobotState::CALIBRATING:      return "CALIBRATING";
        case RobotState::IDLE_ARMED:       return "IDLE_ARMED";
        case RobotState::LINE_FOLLOW:      return "LINE_FOLLOW";
        case RobotState::TURNING:          return "TURNING";
        case RobotState::OBSTACLE_AVOID:   return "OBSTACLE_AVOID";
        case RobotState::RAMP_TRANSIT:     return "RAMP_TRANSIT";
        case RobotState::ZONE_ENTERED:     return "ZONE_ENTERED";
        case RobotState::MINE_DEFUSE:      return "MINE_DEFUSE";
        case RobotState::VICTIM_SCAN:      return "VICTIM_SCAN";
        case RobotState::VICTIM_ALIGN:     return "VICTIM_ALIGN";
        case RobotState::VICTIM_APPROACH:  return "VICTIM_APPROACH";
        case RobotState::VICTIM_GRIP:      return "VICTIM_GRIP";
        case RobotState::VICTIM_LIFT:      return "VICTIM_LIFT";
        case RobotState::EXIT_SEEK:        return "EXIT_SEEK";
        case RobotState::EXIT_HOLD:        return "EXIT_HOLD";
        case RobotState::LACK_OF_PROGRESS: return "LACK_OF_PROGRESS";
        case RobotState::FINISHED:         return "FINISHED";
        case RobotState::FAULT:            return "FAULT";
    }
    return "?";
}

bool isMoving(RobotState s) {
    return s != RobotState::BOOT && s != RobotState::CALIBRATING &&
           s != RobotState::IDLE_ARMED && s != RobotState::LACK_OF_PROGRESS &&
           s != RobotState::FINISHED && s != RobotState::FAULT;
}

void out(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void out(const char* fmt, ...) {
    char buf[700];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n >= static_cast<int>(sizeof(buf))) n = sizeof(buf) - 1;
    netSend(buf, static_cast<size_t>(n));
}

void fault(const char* why) {
    pinMode(Pins::MOTOR_STBY, OUTPUT);
    digitalWrite(Pins::MOTOR_STBY, LOW);   // drivers off, hard (as haltWithFault)
    g_fault = why;
    Serial.print(F("[FATAL] "));
    Serial.println(why);
}

void pressStart() {
    // Open-drain: can only pull LOW, never drive HIGH against a pressed button.
    pinMode(Pins::START_BUTTON, OUTPUT_OPEN_DRAIN);
    digitalWrite(Pins::START_BUTTON, LOW);
    g_pressUntil = millis() + VIRTUAL_PRESS_MS;
    if (g_pressUntil == 0) g_pressUntil = 1;
}

void restartInto(uint8_t mode) {
    if (g_tasksUp) g_tasks.emergencyStop();
    digitalWrite(Pins::MOTOR_STBY, LOW);
    Preferences p;
    p.begin("bench", false);
    p.putUChar("mode", mode);
    p.end();
    out("{\"ev\":\"restarting\",\"mode\":\"%s\"}", mode ? "run" : "bench");
    netService();
    delay(150);
    ESP.restart();
}

void hello() {
    out("{\"ev\":\"hello\",\"mode\":\"run\",\"fw\":\"syrobix-bench-wifi\",\"ver\":1,\"ip\":\"%s\","
        "\"net\":\"%s\",\"fault\":\"%s\",\"linkstop\":%d,\"linkstop_ms\":%lu}",
        netIp(), netMode(), g_fault, g_linkStop, static_cast<unsigned long>(LINK_LOSS_STOP_MS));
}

void onText(char* line) {
    g_lastHostMs = millis();
    for (char* q = line; *q; ++q) *q = static_cast<char>(toupper(*q));
    if (!strcmp(line, "HB")) return;
    if (!strcmp(line, "HELLO")) { hello(); return; }
    if (!strcmp(line, "STOP")) {
        if (g_tasksUp) g_tasks.emergencyStop();
        digitalWrite(Pins::MOTOR_STBY, LOW);
        g_userStopped = true;
        out("{\"ev\":\"stop\",\"why\":\"cmd\"}");
        return;
    }
    if (!strcmp(line, "START")) {
        if (!g_tasksUp) { out("{\"ev\":\"err\",\"cmd\":\"START\",\"msg\":\"firmware not running\"}"); return; }
        pressStart();
        out("{\"ev\":\"ack\",\"cmd\":\"START\"}");
        return;
    }
    if (!strncmp(line, "LINKSTOP ", 9)) {
        g_linkStop = line[9] != '0';
        out("{\"ev\":\"ack\",\"cmd\":\"LINKSTOP\"}");
        return;
    }
    if (!strcmp(line, "MODE BENCH")) { restartInto(0); return; }
    if (!strcmp(line, "MODE RUN"))   { restartInto(1); return; }
    out("{\"ev\":\"err\",\"cmd\":\"%s\",\"msg\":\"run mode: HB HELLO START STOP LINKSTOP MODE\"}", line);
}

void onAllGone() {
    // The page closed. The link-loss rule below handles a running robot.
}

void sendStatus(uint32_t now) {
    if (!g_tasksUp) {
        out("{\"ty\":\"run\",\"t\":%lu,\"name\":\"%s\",\"fault\":\"%s\"}",
            static_cast<unsigned long>(now), *g_fault ? "BOOT_FAULT" : "BOOT", g_fault);
        return;
    }
    const MissionContext& c = g_tasks.context();
    SensorSnapshot s = SensorSnapshot();
    const bool snapOk = g_sensors.getSnapshot(s, 5);
    const unsigned long matchMs = c.matchStartMs ? static_cast<unsigned long>(now - c.matchStartMs) : 0UL;
    out("{\"ty\":\"run\",\"t\":%lu,\"st\":%u,\"name\":\"%s\",\"prev\":\"%s\",\"in\":%lu,\"match\":%lu,"
        "\"zone\":%d,\"mines\":%u,\"vic\":%d,\"deliv\":%d,\"att\":%u,\"lop\":%u,\"score\":%u,"
        "\"estop\":%d,\"linkstopped\":%d,\"userstop\":%d,\"btn\":%d,\"snap\":%d,"
        "\"line\":%.2f,\"lost\":%d,\"act\":%u,\"yaw\":%.1f,\"pitch\":%.1f,\"ramp\":%d,\"imu\":%d,"
        "\"fr\":%u,\"lr\":%u,\"rr\":%u,\"cl\":%u,\"cr\":%u,\"fault\":\"%s\"}",
        static_cast<unsigned long>(now), static_cast<unsigned>(c.state), stateName(c.state),
        stateName(c.previousState), static_cast<unsigned long>(now - c.stateEnteredMs), matchMs,
        c.zoneEntered, c.minesDefused, c.victimAcquired, c.victimDelivered, c.victimAttempts,
        c.lackOfProgressCount, c.estimatedScore, c.emergencyStopped, g_linkStopped, g_userStopped,
        digitalRead(Pins::START_BUTTON) == LOW, snapOk,
        snapOk ? s.linePosition : 0.0f, snapOk && s.lineLost, snapOk ? s.lineActiveCount : 0,
        snapOk ? s.yawDeg : 0.0f, snapOk ? s.pitchDeg : 0.0f, snapOk && s.onRamp, snapOk && s.imuHealthy,
        snapOk ? s.rangeFrontMm : 0, snapOk ? s.rangeLeftMm : 0, snapOk ? s.rangeRightMm : 0,
        snapOk ? static_cast<unsigned>(s.colorLeft) : 0u, snapOk ? static_cast<unsigned>(s.colorRight) : 0u,
        g_fault);
}

}  // namespace

void runSetup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println(F("=== SYROBIX RUN mode (Wi-Fi practice build) — NOT FOR COMPETITION ==="));

    // ---- Same order as src/main.cpp ----------------------------------------
    if (!g_drivetrain.begin()) {
        fault("drivetrain init failed");
    } else if (!g_sensors.begin()) {
        LidarBank& tof = g_sensors.lidar();
        if (tof.busFault())                    fault("ToF: a device answers 0x29 with every XSHUT low");
        else if (!tof.isUp(LidarId::FRONT))    fault("FRONT VL53L1X init failed (power / XSHUT / I2C)");
        else                                   fault("MPU6500 not answering WHO_AM_I (I2C / wiring)");
    } else if (!g_tasks.begin()) {
        fault(g_tasks.peripheralsUp() ? "task creation failed (out of heap?)"
                                      : "PCA9685 not answering at 0x40");
    } else {
        g_tasksUp = true;
        Serial.println(F("[run] calibrating — sweep the array across the line, then follow the beeps"));
    }

    // Wi-Fi last, so the match bring-up timing is unchanged. The radio lives on
    // core 0 next to the sensor/mission tasks, which Config.h's RTOS layout
    // already reserves room for (Rtos:: comment, "hosts the Wi-Fi/BT stack").
    netBegin(onText, onAllGone);
    Serial.printf("[run] Wi-Fi %s %s\n", netMode(), netIp());
}

void runLoop() {
    // Arduino loop task: core 1, priority 1 — below the control (5) and line
    // (4) tasks, and it never touches I2C (getSnapshot is a mutex copy).
    const uint32_t now = millis();
    netService();

    if (g_pressUntil && static_cast<int32_t>(now - g_pressUntil) >= 0) {
        pinMode(Pins::START_BUTTON, INPUT_PULLUP);   // release, as TaskHandler configured it
        g_pressUntil = 0;
    }

    if (g_tasksUp && g_linkStop && !g_linkStopped && isMoving(g_tasks.context().state) &&
        g_lastHostMs != 0 && now - g_lastHostMs > LINK_LOSS_STOP_MS) {   // only once a phone was connected
        g_tasks.emergencyStop();
        g_linkStopped = true;
        Serial.println(F("[run] link lost -> emergency stop"));
    }

    if (now - g_lastStatus >= STATUS_PERIOD_MS) {
        g_lastStatus = now;
        sendStatus(now);
    }
    delay(2);   // yield; keeps the loop task from spinning
}
