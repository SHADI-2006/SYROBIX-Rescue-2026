#pragma once
#include <stdint.h>
#include <stddef.h>
class TwoWire { public:
 bool begin(int sda=-1,int scl=-1,uint32_t f=0);
 void beginTransmission(uint8_t); uint8_t endTransmission(bool stop=true);
 size_t write(uint8_t); uint8_t requestFrom(uint8_t,uint8_t); int read(); int available(); void setClock(uint32_t); size_t write(const uint8_t*, size_t); uint8_t endTransmission(uint8_t); size_t requestFrom(uint8_t,size_t,bool); size_t readBytes(uint8_t*, size_t);  };
extern TwoWire Wire;
