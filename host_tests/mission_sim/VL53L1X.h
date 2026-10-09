#pragma once
#include <stdint.h>
#include "Wire.h"
class VL53L1X { public:
  enum DistanceMode { Short, Medium, Long, Unknown };
  enum RangeStatus : uint8_t { RangeValid=0, RangeValidMinRangeClipped=3, None=255 };
  struct RangingData { uint16_t range_mm; RangeStatus range_status; };
  RangingData ranging_data{}; uint8_t last_status=2;
  void setBus(TwoWire*){} void setTimeout(uint16_t){} bool init(bool=true){return false;}
  void setAddress(uint8_t){} bool setDistanceMode(DistanceMode){return false;}
  bool setMeasurementTimingBudget(uint32_t){return false;} void startContinuous(uint32_t){}
  void stopContinuous(){} bool dataReady(){return false;} uint16_t read(bool=true){return 0;} };
