// Host simulation of the REAL TaskHandler state machine (TaskHandler.cpp is
// included verbatim). A tiny kinematic world integrates the drive mailbox.
#include "TaskHandler.cpp"
#include <stdio.h>
#include <functional>
#include <string>
extern uint32_t g_ms; extern int g_button;
#include <vector>
#include <algorithm>

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("   FAIL: %s  (line %d)\n", msg, __LINE__); ++fails; } } while (0)

struct World {
    float dist = 0, yaw = 0;
    RobotDrivetrain drive; SensorsSystem sensors; TaskHandler th{drive, sensors};
    std::function<void(SensorSnapshot&, World&)> paint;
    int maxStates = 0;
    std::string trace;
    uint32_t heldStamp = 0xFFFFFFFF; TileColor heldL = TileColor::WHITE, heldR = TileColor::WHITE;
    void setOdo() {
        const int32_t c = (int32_t)(dist / Hw::MM_PER_COUNT);
        drive._encLeft._count = c; drive._encRight._count = c;
    }
    void motion(float dt) {           // what the Core-1 control task would do
        float l, r;
        if (g_lineFollowEnabled) { l = r = g_commandedSpeed; }
        else {
            const uint32_t c = g_driveCmd;
            if ((c >> 22) == CMD_TANK) { l = (int)((c >> 11) & 0x7FF) - 1024; r = (int)(c & 0x7FF) - 1024; }
            else { l = r = 0; }
        }
        dist += 0.5f * (l + r) * (270.0f / 1023.0f) * dt;
        yaw   = wrapDeg(yaw + 0.5f * (r - l) * (150.0f / 420.0f) * dt);
    }
    RobotState st() const { return th._ctx.state; }
    void run(uint32_t ms) {
        for (uint32_t t = 0; t < ms; t += 20) {
            setOdo();
            SensorSnapshot s = SensorSnapshot();
            s.imuHealthy = true; s.yawDeg = yaw; s.lineLost = false; s.lineActiveCount = 1;
            s.rangeFrontMm = s.rangeLeftMm = s.rangeRightMm = Tune::TOF_INVALID;
            s.rangeFrontStampMs = g_ms; s.timestampMs = g_ms - (g_ms % 30);
            s.colorLeft = s.colorRight = TileColor::WHITE;
            if (paint) paint(s, *this);
            // Real pipeline: colour is produced once per ~30 ms frame and HELD.
            if (s.timestampMs != heldStamp) { heldStamp = s.timestampMs; heldL = s.colorLeft; heldR = s.colorRight; }
            s.colorLeft = heldL; s.colorRight = heldR;
            const RobotState before = st();
            PcaBus::service(g_ms);
            th._indicator.update();
            th._manipulator.update(0.02f);
            th.tickStateMachine(s, 0.02f);
            if (st() != before) { trace += std::to_string((int)st()) + " "; }
            motion(0.02f); g_ms += 20;
        }
    }
    void startIn(RobotState s, bool carrying = false) {
        th._ctx.matchStartMs = g_ms; th._ctx.victimAcquired = carrying;
        g_redLockoutUntilMm = -1e9f; g_greenIgnoreUntilMm = -1e9f;
        th._ctx.state = RobotState::BOOT; th.transitionTo(s);
    }
};
static const char* N(RobotState s){ static const char* n[]={"BOOT","CALIB","IDLE","LINE","TURN","OBST","RAMP","ZONE","MINE","SCAN","ALIGN","APPR","GRIP","LIFT","EXIT_SEEK","EXIT_HOLD","LOP","FIN","FAULT"}; return (unsigned)s<19?n[(int)s]:"?"; }

int main() {
    // ---------------------------------------------------------------- C4 + C5
    { printf("[C4] 4-way crossing, LEFT marker -> must actually turn left\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      float yawAtPivot = 999; bool pivoting = false;
      w.paint = [&](SensorSnapshot& s, World& W){
        if (W.dist > 100 && W.dist < 125) s.colorLeft = TileColor::GREEN;
        if (W.dist > 150 && W.dist < 165) s.lineActiveCount = 6;
        if (W.st()==RobotState::TURNING && g_phase==2) {
            if (!pivoting) { pivoting = true; yawAtPivot = W.yaw; }
            const float t = fabsf(wrapDeg(W.yaw - yawAtPivot));
            s.lineLost = (t > 20 && t < 75);      // leaves straight arm, finds left arm at ~75+
            s.linePosition = (t >= 75) ? 0.1f : 0.0f;
        }};
      w.run(4000);
      const float turned = wrapDeg(w.yaw - yawAtPivot);
      printf("   states: %s | turned %+.1f deg\n", w.trace.c_str(), turned);
      CHECK(w.th._ctx.pendingTurn == TurnDirection::NONE && w.st()==RobotState::LINE_FOLLOW, "back in LINE_FOLLOW");
      CHECK(turned > 60, "rotated LEFT onto the branch (old code: 0 deg, went straight)"); }

    { printf("[C5] dead-end marker pair with 8 mm skew -> U-turn\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      w.paint = [&](SensorSnapshot& s, World& W){
        if (W.dist > 100 && W.dist < 125) s.colorLeft  = TileColor::GREEN;
        if (W.dist > 108 && W.dist < 133) s.colorRight = TileColor::GREEN;
        if (W.dist > 150 && W.dist < 165) s.lineActiveCount = 6; };
      w.run(1500);
      printf("   pendingTurn=%d state=%s\n", (int)w.th._ctx.pendingTurn, N(w.st()));
      CHECK(w.st()==RobotState::TURNING && w.th._ctx.pendingTurn==TurnDirection::U_TURN, "U_TURN latched (old code: LEFT)"); }

    { printf("[C5] marker AFTER the bar belongs to another approach -> straight\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      w.paint = [&](SensorSnapshot& s, World& W){
        if (W.dist > 150 && W.dist < 165) s.lineActiveCount = 6;
        if (W.dist > 170 && W.dist < 195) s.colorRight = TileColor::GREEN; };
      w.run(2500);
      printf("   states: '%s' dist=%.0f\n", w.trace.c_str(), w.dist);
      CHECK(w.st()==RobotState::LINE_FOLLOW && w.trace.empty(), "no turn taken"); }

    { printf("[C5] single noisy green frame, no bar -> ignored\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      w.paint = [&](SensorSnapshot& s, World& W){ if (W.dist > 100 && W.dist < 104) s.colorLeft = TileColor::GREEN;
                                                   if (W.dist > 400 && W.dist < 415) s.lineActiveCount = 6; };
      w.run(2500);
      CHECK(w.trace.empty(), "no phantom turn at the later bar"); }

    // ------------------------------------------------------------------- C8
    { printf("[C8] carrying: mine tile on the way out, then the red stripe\n");
      World w; w.startIn(RobotState::EXIT_SEEK, true);
      w.paint = [&](SensorSnapshot& s, World& W){
        const bool red = (W.dist > 100 && W.dist < 400) || (W.dist > 900 && W.dist < 915);
        if (red) s.colorLeft = s.colorRight = TileColor::RED; };
      w.run(26000);
      printf("   states: %s | mines=%u delivered=%d dist=%.0f\n", w.trace.c_str(), w.th._ctx.minesDefused, w.th._ctx.victimDelivered, w.dist);
      CHECK(w.th._ctx.minesDefused == 1, "mine tile defused, NOT treated as the exit");
      CHECK(w.th._ctx.victimDelivered && w.st()==RobotState::FINISHED, "stripe ended the match");
      CHECK(w.dist > 900 && w.dist < 960, "stopped at the stripe, not on the mine"); }

    { printf("[C8] single misclassified red frame while carrying -> no exit\n");
      World w; w.startIn(RobotState::EXIT_SEEK, true);
      w.paint = [&](SensorSnapshot& s, World& W){ if (W.dist > 100 && W.dist < 103) s.colorLeft = s.colorRight = TileColor::RED; };
      w.run(1500);
      printf("   states: '%s'\n", w.trace.c_str());
      CHECK(w.st()==RobotState::EXIT_SEEK, "still seeking"); }

    // ------------------------------------------------------------------- H4
    { printf("[H4] classifier stuck on red for 1.2 m -> defusals separated by re-arm distance\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      std::vector<float> at;
      w.paint = [&](SensorSnapshot& s, World& W){ if (W.dist > 100 && W.dist < 1300) s.colorLeft = s.colorRight = TileColor::RED;
                                                   if (W.st()==RobotState::MINE_DEFUSE && g_phase==2 && (at.empty() || W.dist - at.back() > 1)) at.push_back(W.dist); };
      w.run(40000);
      printf("   dwell positions:"); for (float a : at) printf(" %.0f", a); printf("\n");
      bool spaced = true; for (size_t i = 1; i < at.size(); ++i) if (at[i]-at[i-1] < Mission::MINE_REARM_MM) spaced = false;
      CHECK(at.size() >= 2 && spaced, "no same-spot livelock; each dwell >= MINE_REARM_MM apart");
      CHECK(!at.empty() && at[0] > 100 + Mission::RED_TILE_CONFIRM_MM + 50, "first dwell advanced onto the tile centre"); }

    // --------------------------------------------------------------- C6 + H3
    { printf("[C6] 'obstacle' that is actually the victim on the safe tile\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      w.paint = [&](SensorSnapshot& s, World& W){
        if (W.dist > 100) { s.obstacleAhead = W.st()==RobotState::LINE_FOLLOW; s.rangeFrontMm = 110; }
        if (W.dist > 105) s.colorLeft = s.colorRight = TileColor::GREEN; };
      w.run(1500);
      printf("   states: %s\n", w.trace.c_str());
      CHECK(w.trace.find("5 7 9") == 0, "LINE -> OBSTACLE(verify) -> ZONE -> SCAN"); }

    { printf("[H3] victim never found -> bounded retries, then lack of progress\n");
      World w; w.startIn(RobotState::VICTIM_SCAN);
      w.run(90000);
      printf("   states: %s attempts=%u\n", w.trace.c_str(), w.th._ctx.victimAttempts);
      CHECK(w.st()==RobotState::LACK_OF_PROGRESS && w.th._ctx.victimAttempts==Mission::MAX_VICTIM_ATTEMPTS, "gave up after MAX attempts"); }

    { printf("[C3] stop-and-go scan finds a victim at +20 deg and aligns to it\n");
      World w; w.startIn(RobotState::VICTIM_SCAN);
      w.paint = [&](SensorSnapshot& s, World& W){
        const float e = fabsf(wrapDeg(W.yaw - 20.0f));
        if (e < 12.0f) s.rangeFrontMm = (uint16_t)(150 + e * 4 - W.dist); };
      w.run(20000);
      printf("   states: %s bearing=%.1f yaw=%.1f\n", w.trace.c_str(), w.th._ctx.victimBearingDeg, w.yaw);
      CHECK(fabsf(wrapDeg(w.th._ctx.victimBearingDeg - 20.0f)) < 8.0f, "bearing within one scan step");
      CHECK(w.trace.find("10 11 12 13 4") == 0, "SCAN -> ALIGN -> APPROACH -> GRIP -> LIFT -> U-TURN");
      CHECK(w.th._ctx.victimAcquired, "victim aboard"); }


    { printf("[C7] real obstacle: bypass right, return TOWARD the line on the left\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      float minYaw = 0, maxYaw = 0; float d14 = -1;
      w.paint = [&](SensorSnapshot& s, World& W){
        if (W.st()==RobotState::LINE_FOLLOW && W.dist > 100 && W.dist < 110) { s.obstacleAhead = true; s.rangeFrontMm = 115; }
        if (W.st()==RobotState::OBSTACLE_AVOID) {
            if (W.yaw < minYaw) minYaw = W.yaw; if (W.yaw > maxYaw) maxYaw = W.yaw;
            if (g_phase == 14 && d14 < 0) d14 = W.dist;
            s.lineLost = (g_phase >= 11) && !(g_phase == 14 && W.dist - d14 > 150);
        } };
      w.run(12000);
      printf("   states: %s | yaw swung %.0f .. %+.0f\n", w.trace.c_str(), minYaw, maxYaw);
      CHECK(w.trace.find("5 3") == 0, "OBSTACLE_AVOID -> back to LINE_FOLLOW");
      CHECK(minYaw <= -Mission::OBSTACLE_TURN_DEG + 2, "first swung RIGHT (out)");
      CHECK(maxYaw >= Mission::OBSTACLE_RETURN_DEG - 2, "then converged LEFT, toward the line (old arc curved away)"); }

    { printf("[H1] IMU dies mid-run -> U-turn still terminates on encoder heading\n");
      World w; w.startIn(RobotState::TURNING); w.th._ctx.pendingTurn = TurnDirection::U_TURN;
      w.th._ctx.state = RobotState::BOOT; w.th.transitionTo(RobotState::TURNING);
      w.paint = [&](SensorSnapshot& s, World& W){ s.imuHealthy = false; s.lineLost = false;
        W.drive._headingDeg = W.yaw * 0.65f; };   // skid-steer: encoders under-read ~35 %
      w.run(9000);
      printf("   states: %s | real yaw %.0f\n", w.trace.c_str(), w.yaw);
      CHECK(w.trace.find("3") == 0, "U-turn completed (overshoots, but does not hang)"); }


    // ======================================================== Rev 3: PCA9685
    { printf("[P0] PCA9685 absent at boot -> begin() fails, reported as peripheral fault\n");
      g_chip = FakePca(); g_chip.present = false;
      World w; const bool ok = w.th.begin();
      CHECK(!ok && !w.th.peripheralsUp(), "begin() false and peripheralsUp() false (main.cpp fault 6)"); }

    { printf("[P1] boot: 50 Hz prescale cached, poses sent as exact tick counts\n");
      g_chip = FakePca(); g_pcaUp = false; g_pcaPrescale = 0;
      World w; CHECK(w.th.begin(), "begin() ok");
      auto last = [](uint8_t ch) { PcaEvent e{0,ch,0xFFFF,0xFFFF}; for (auto& x : g_chip.log) if (x.ch == ch) e = x; return e; };
      const auto arm = last(PcaChannels::SERVO_ARM), grip = last(PcaChannels::SERVO_GRIPPER);
      const auto led = last(PcaChannels::STATUS_LED), bz = last(PcaChannels::BUZZER);
      printf("   prescale=%u  arm off=%u  grip off=%u  led(on,off)=(%u,%u) buzzer=(%u,%u)\n",
             g_chip.prescale, arm.off, grip.off, led.on, led.off, bz.on, bz.off);
      CHECK(g_chip.prescale == 131 && !(g_chip.mode1 & 0x10), "27 MHz / 50 Hz -> prescale 131, awake");
      CHECK(arm.on == 0 && arm.off == 264,  "arm CARRY 75 deg -> 1291 us -> 264 ticks");
      CHECK(grip.on == 0 && grip.off == 318, "gripper OPEN 100 deg -> 1555 us -> 318 ticks");
      CHECK(bz.on == 0 && bz.off == 4096, "buzzer initialised to true full-OFF (bit 12)");
      CHECK(led.off == 4096 || led.on == 4096, "LED uses full-ON/full-OFF codes only"); }

    { printf("[P2] slew limiter: identical to the Rev 2 algorithm, tick for tick\n");
      g_chip = FakePca(); g_pcaUp = false;
      World w; w.th.begin();
      Manipulator& m = w.th._manipulator;
      struct Ref { uint8_t cur, tgt; float acc = 0;          // ORIGINAL Rev 2 code, verbatim
        void update(float dt) { acc += SERVO_DEG_PER_SEC * dt; if (acc < 1.0f) return;
          const int16_t step = (int16_t)acc; acc -= (float)step;
          if (cur != tgt) { const int16_t d = (int16_t)tgt - cur;
            const int16_t mv = (abs(d) < step) ? d : (d > 0 ? step : -step); cur = (uint8_t)(cur + mv); } } };
      Ref ra{Hw::ARM_ANGLE_CARRY, Hw::ARM_ANGLE_DOWN}, rg{Hw::GRIPPER_OPEN, Hw::GRIPPER_CLOSED};
      m.armDown(); m.closeGripper();
      const float dts[] = {0.020f, 0.019f, 0.023f, 0.004f, 0.031f, 0.020f, 0.0005f, 0.050f};
      bool same = true; int ticks = 0;
      for (int i = 0; i < 200; ++i) {
          const float dt = dts[i % 8]; m.update(dt); ra.update(dt); rg.update(dt); ++ticks;
          if (m.armAngle() != ra.cur || m.gripperAngle() != rg.cur) { same = false; break; }
          if (i == 60) { m.armToCarry(); ra.tgt = Hw::ARM_ANGLE_CARRY; m.openGripper(); rg.tgt = Hw::GRIPPER_OPEN; }
      }
      PcaEvent armLast{}; for (auto& x : g_chip.log) if (x.ch == PcaChannels::SERVO_ARM) armLast = x;
      CHECK(same, "every intermediate angle matches the original slew");
      CHECK(m.isSettled() && armLast.off == 264, "settled back at CARRY, and the PCA holds that pulse"); }

    { printf("[P3] scored mine chain on the PCA: dark dwell -> 3 blinks -> beep, in order\n");
      g_chip = FakePca(); g_pcaUp = false;
      World w; w.th.begin(); w.startIn(RobotState::LINE_FOLLOW);
      uint32_t dwellStart = 0;
      w.paint = [&](SensorSnapshot& s, World& W){
        if (W.dist > 100 && W.dist < 400) s.colorLeft = s.colorRight = TileColor::RED;
        if (W.st() == RobotState::MINE_DEFUSE && g_phase == 2 && !dwellStart) dwellStart = g_phaseStartMs; };
      g_chip.log.clear();
      w.run(16000);
      std::vector<PcaEvent> led, bz;
      for (auto& e : g_chip.log) if (e.ms >= dwellStart && dwellStart) {
          if (e.ch == PcaChannels::STATUS_LED) led.push_back(e);
          if (e.ch == PcaChannels::BUZZER)     bz.push_back(e); }
      auto isOn = [](const PcaEvent& e){ return e.on == 4096; };
      int onsBeforeBeep = 0; uint32_t firstOn = 0, lastLedOff = 0;
      const uint32_t beepOn = bz.empty() ? 0 : bz[0].ms;
      for (auto& e : led) { if (e.ms > beepOn && beepOn) break;
          if (isOn(e)) { if (!firstOn) firstOn = e.ms; ++onsBeforeBeep; } else lastLedOff = e.ms; }
      printf("   dwell@%u  first blink +%u ms  blinks=%d  last LED off@+%u  beep %u..%u (%u ms)\n",
             dwellStart, firstOn - dwellStart, onsBeforeBeep, lastLedOff - dwellStart,
             bz.size() > 0 ? bz[0].ms - dwellStart : 0, bz.size() > 1 ? bz[1].ms - dwellStart : 0,
             bz.size() > 1 ? bz[1].ms - bz[0].ms : 0);
      CHECK(dwellStart && firstOn - dwellStart >= Mission::MINE_DWELL_MS, "LED dark for the full 5 s dwell");
      CHECK(onsBeforeBeep == 3, "exactly three blinks");
      CHECK(bz.size() >= 2 && isOn(bz[0]) && !isOn(bz[1]) && bz[0].ms >= lastLedOff, "beep only after the third blink");
      CHECK(bz.size() >= 2 && bz[1].ms - bz[0].ms >= Mission::MINE_BUZZER_MS &&
            bz[1].ms - bz[0].ms <= Mission::MINE_BUZZER_MS + 40, "beep length = MINE_BUZZER_MS (one 20 ms tick)"); }

    { printf("[P4] status LED writes are change-driven, not 50 per second\n");
      g_chip = FakePca(); g_pcaUp = false;
      World w; w.th.begin(); w.startIn(RobotState::LINE_FOLLOW); g_chip.log.clear();
      w.run(10000);
      int n = 0; for (auto& e : g_chip.log) if (e.ch == PcaChannels::STATUS_LED) ++n;
      const int toggles = 2 * (10000 / Tune::LED_PATTERN_RUN_PERIOD_MS) + 2;
      printf("   %d LED writes in 10 s (pattern toggles <= %d, mission ticks = 500)\n", n, toggles);
      CHECK(n <= toggles, "one I2C write per toggle"); }

    { printf("[P5] servo-spike brown-out resets the PCA mid-run -> detected, re-initialised, state re-sent\n");
      g_chip = FakePca(); g_pcaUp = false;
      World w; w.th.begin(); w.startIn(RobotState::LINE_FOLLOW); w.run(1500);
      const uint32_t inits0 = g_chip.inits; const bool ledNow = w.th._indicator._ledWritten;
      g_chip.brownOut();
      w.run(Hw::PCA_HEALTH_PERIOD_MS + 100);
      printf("   inits %u -> %u, prescale=%u, arm off=%u grip off=%u\n", inits0, g_chip.inits, g_chip.prescale, g_chip.off[0], g_chip.off[1]);
      CHECK(g_chip.inits == inits0 + 1 && g_chip.prescale == 131 && !(g_chip.mode1 & 0x10), "re-initialised once, awake at 50 Hz");
      CHECK(g_chip.off[PcaChannels::SERVO_ARM] == 264 && g_chip.off[PcaChannels::SERVO_GRIPPER] == 318, "both servo poses re-sent");
      CHECK((g_chip.on[PcaChannels::STATUS_LED] == 4096) == w.th._indicator._ledWant, "LED state re-sent");
      (void)ledNow; }

    { printf("[P6] a NACKed write is retried, not assumed\n");
      g_chip = FakePca(); g_pcaUp = false;
      World w; w.th.begin(); w.startIn(RobotState::LINE_FOLLOW); w.run(100);
      g_chip.failNext = 1; w.th._indicator.beep(300);
      const bool afterFail = g_chip.on[PcaChannels::BUZZER] == 4096;
      w.run(40);
      CHECK(!afterFail && g_chip.on[PcaChannels::BUZZER] == 4096, "buzzer ON lands on the retry tick"); }

    // ------------------------------------------------------------------- C1
    { printf("[C1] every stop path posts BRAKE and disables line following\n");
      World w; w.startIn(RobotState::LINE_FOLLOW);
      CHECK(g_lineFollowEnabled, "following");
      w.th.transitionTo(RobotState::ZONE_ENTERED);
      CHECK(!g_lineFollowEnabled && (g_driveCmd >> 22) == CMD_BRAKE, "brake posted + follow off");
      w.th.transitionTo(RobotState::FINISHED);
      CHECK(g_driveKill, "FINISHED kills the drive on Core 1"); g_driveKill = false; }

    printf("\n%s (%d failure%s)\n", fails ? "SIMULATION FAILED" : "ALL SCENARIOS PASS", fails, fails==1?"":"s");
    return fails ? 1 : 0;
}
