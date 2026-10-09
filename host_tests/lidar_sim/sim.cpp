#include "LidarBank.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <stdio.h>
#include <assert.h>
TwoWire Wire; uint32_t g_ms=1000; uint32_t VL53L1X::lastPeriod=0;
FakeDev g_dev[3];
unsigned long millis(){return g_ms;} unsigned long micros(){return g_ms*1000;}
void delay(uint32_t m){g_ms+=m;} void delayMicroseconds(uint32_t){} void vTaskDelay(TickType_t t){g_ms+=t;}
static int pinMode_[64]; static int pinLvl[64];
void pinMode(uint8_t p,uint8_t m){pinMode_[p]=m;}
void digitalWrite(uint8_t p,uint8_t v){pinLvl[p]=v;}
int digitalRead(uint8_t p){return pinLvl[p];}
static void physics(){ // XSHUT -> power. INPUT == released (board pull-up)
  for(auto&d:g_dev){ bool high = d.xshutBroken || (pinMode_[d.xshut]==INPUT) || (pinMode_[d.xshut]==OUTPUT && pinLvl[d.xshut]);
    if(!high){ d.powered=false; d.addr=0x29; d.ranging=false; } else if(!d.powered){ d.powered=true; d.addr=0x29; } } }
int devAt(uint8_t a){ physics(); int f=-1,n=0; for(int i=0;i<3;i++) if(g_dev[i].powered&&g_dev[i].addr==a){f=i;n++;} return n>1?-2:f; }
uint8_t TwoWire::endTransmission(bool){ int d=devAt(cur); return d==-1?2:0; }
void VL53L1X::startContinuous(uint32_t p){ lastPeriod=p; int d=devAt(address); if(d>=0){g_dev[d].ranging=true; g_dev[d].nextReady=g_ms+p;} }
bool VL53L1X::dataReady(){ int d=devAt(address); if(d<0){last_status=2;return false;} last_status=0; return g_dev[d].ranging && g_ms>=g_dev[d].nextReady; }
uint16_t VL53L1X::read(bool){ int d=devAt(address); if(d<0){last_status=2;return 0;} last_status=0; g_dev[d].nextReady=g_ms+38;
  ranging_data.range_mm=g_dev[d].mm; ranging_data.range_status=(RangeStatus)g_dev[d].st; return g_dev[d].mm; }
static void reset(bool warm){ for(int i=0;i<3;i++){ uint8_t a=g_dev[i].addr; bool pw=g_dev[i].powered; g_dev[i]=FakeDev{ (uint8_t)(40+i), warm?pw:true, warm?a:(uint8_t)0x29, false,0,(uint16_t)(100+i*50),0,false}; }
  for(int p=0;p<64;p++){pinMode_[p]=INPUT;pinLvl[p]=0;} }
int main(){
  // 1) cold boot: all three powered on 0x29 (pins float = released)
  reset(false); { LidarBank b; bool ok=b.begin(); assert(ok && b.upMask()==0x7 && !b.busFault());
    assert(g_dev[0].addr==Hw::TOF_ADDR_FRONT && g_dev[1].addr==Hw::TOF_ADDR_LEFT && g_dev[2].addr==Hw::TOF_ADDR_RIGHT);
    // non-blocking service, staleness, status handling
    b.updateNext(); assert(!b.isValid(LidarId::FRONT));            // nothing ready yet: no wait
    g_ms+=30; b.updateNext(); g_ms+=40; b.updateNext(); assert(b.rangeMm(LidarId::FRONT)==100 && b.rangeMm(LidarId::RIGHT)==200);
    g_dev[0].st=2; g_ms+=40; b.updateNext(); assert(b.rangeMm(LidarId::FRONT)==Tune::TOF_INVALID);   // SignalFail rejected
    g_dev[0].st=3; g_dev[0].mm=30; g_ms+=40; b.updateNext(); assert(b.rangeMm(LidarId::FRONT)==30);   // clipped accepted
    b.setFocus(LidarId::FRONT); for(int k=0;k<10;k++){g_ms+=30; b.updateNext();}
    assert(b.isValid(LidarId::FRONT) && !b.isValid(LidarId::LEFT));  // sides go stale when focused
    b.clearFocus(); g_ms+=30; b.updateNext(); assert(!b.isValid(LidarId::LEFT)); // stale sample discarded on resume
    g_ms+=40; b.updateNext(); assert(b.isValid(LidarId::LEFT));
    assert(b.setTimingBudgetMs(LidarId::LEFT,20)); g_ms+=30; b.updateNext();
    assert(!b.setTimingBudgetMs(LidarId::LEFT,10)); assert(b.setDistanceMode(LidarId::LEFT,TofDistanceMode::LONG));
    g_ms+=30; b.updateNext(); assert(VL53L1X::lastPeriod>=38);   // LONG forced budget>=33
    printf("cold boot + runtime: PASS\n");
    // 2) warm reset: devices keep 0x30..0x32, MCU re-runs begin()
    reset(true); bool ok2=b.begin(); assert(ok2 && b.upMask()==0x7); printf("warm reset re-address: PASS\n"); }
  // 3) broken XSHUT on LEFT: stays powered on 0x29 -> must refuse, not collide
  reset(false); g_dev[1].xshutBroken=true; { LidarBank b; bool ok=b.begin(); assert(!ok && b.busFault()); printf("stuck XSHUT detected, refused: PASS\n"); }
  // 4) dead RIGHT unit: FRONT-only mandatory, run continues
  reset(false); g_dev[2].xshut=63; pinMode_[63]=OUTPUT; pinLvl[63]=0; // wired to nothing -> never powers
  { LidarBank b; bool ok=b.begin(); assert(ok && b.upMask()==0x3); printf("dead side unit tolerated: PASS\n"); }
  return 0; }
