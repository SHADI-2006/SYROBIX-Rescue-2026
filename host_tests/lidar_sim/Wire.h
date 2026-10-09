#pragma once
#include <stdint.h>
#include <stddef.h>
class TwoWire { public:
 void beginTransmission(uint8_t a){cur=a;} uint8_t endTransmission(bool=true);
 size_t write(uint8_t){return 1;} uint8_t requestFrom(uint8_t,uint8_t){return 0;} int read(){return -1;} int available(){return 0;}
 uint8_t cur=0; };
extern TwoWire Wire;
