#pragma once
// Behavioural fake of the Adafruit PCA9685 driver for host simulation.
#include <stdint.h>
#include <vector>
#include "Wire.h"
struct PcaEvent { uint32_t ms; uint8_t ch; uint16_t on, off; };
struct FakePca {                       // the "chip"
    bool present = true; uint8_t mode1 = 0x10; uint8_t prescale = 30;   // power-on: SLEEP, 200 Hz
    uint16_t on[16] = {0}, off[16] = {0}; int failNext = 0; uint8_t regPtr = 0;
    std::vector<PcaEvent> log; uint32_t inits = 0;
    void brownOut() { mode1 = 0x10; prescale = 30; for (int i=0;i<16;++i){on[i]=0;off[i]=4096;} }
};
extern FakePca g_chip; extern uint32_t g_ms;
class Adafruit_PWMServoDriver { public:
  Adafruit_PWMServoDriver(const uint8_t, TwoWire&) {}
  explicit Adafruit_PWMServoDriver(const uint8_t) {}
  bool begin(uint8_t = 0) { if (!g_chip.present) return false; g_chip.mode1 = 0x00; ++g_chip.inits; setPWMFreq(1000); return true; }
  void setOscillatorFrequency(uint32_t f) { osc = f; }
  void setPWMFreq(float freq) { float p = ((osc / (freq * 4096.0f)) + 0.5f) - 1; g_chip.prescale = (uint8_t)p; g_chip.mode1 = 0xA0; }
  uint8_t setPWM(uint8_t ch, uint16_t on, uint16_t off) {
      if (!g_chip.present) return 1;
      if (g_chip.failNext > 0) { --g_chip.failNext; return 1; }
      g_chip.on[ch] = on; g_chip.off[ch] = off; g_chip.log.push_back({g_ms, ch, on, off}); return 0; }
  uint32_t osc = 25000000;
};
