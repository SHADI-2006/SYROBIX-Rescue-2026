#pragma once
#include <stdint.h>
#include "Wire.h"
// ---- Behavioural fake of the Pololu driver + physical devices -------------
struct FakeDev { uint8_t xshut; bool powered; uint8_t addr; bool ranging; uint32_t nextReady; uint16_t mm; uint8_t st; bool xshutBroken; };
extern FakeDev g_dev[3];
int devAt(uint8_t a);   // index of the single powered device at a, -1 none, -2 collision
class VL53L1X { public:
  enum DistanceMode { Short, Medium, Long, Unknown };
  enum RangeStatus : uint8_t { RangeValid=0, SigmaFail=1, SignalFail=2, RangeValidMinRangeClipped=3, None=255 };
  struct RangingData { uint16_t range_mm; RangeStatus range_status; };
  RangingData ranging_data{}; uint8_t last_status=0;
  uint8_t address=0x29; uint16_t to=0; DistanceMode dm=Long; uint32_t budget=50000;
  void setBus(TwoWire*){} void setTimeout(uint16_t t){to=t;}
  bool init(bool=true){ int d=devAt(address); if(d<0){last_status=2;return false;} last_status=0; return to>0; }
  void setAddress(uint8_t a){ int d=devAt(address); if(d>=0) g_dev[d].addr=a; address=a; }
  bool setDistanceMode(DistanceMode m){ if(devAt(address)<0){last_status=2;return false;} dm=m; return true; }
  bool setMeasurementTimingBudget(uint32_t us){ if(us<=4528)return false; budget=us; last_status= devAt(address)<0?2:0; return true; }
  void startContinuous(uint32_t p); void stopContinuous(){ int d=devAt(address); if(d>=0) g_dev[d].ranging=false; }
  bool dataReady(); uint16_t read(bool blocking=true);
  static uint32_t lastPeriod; };
