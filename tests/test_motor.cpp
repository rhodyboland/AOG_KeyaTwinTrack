#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <cstdio>
#include <cassert>
using std::abs;
struct CAN_message_t {uint32_t id=0; struct {bool extended=false;}flags;uint8_t len=0;bool seq=false;uint8_t buf[8]={};};
uint32_t clockNow=0;uint32_t millis(){return clockNow;}
struct FakeBus {std::vector<CAN_message_t> tx;std::vector<CAN_message_t> rx;bool fail=false;void begin(){}void setBaudRate(int){}bool read(CAN_message_t& m){if(rx.empty())return false;m=rx.front();rx.erase(rx.begin());return true;}int write(const CAN_message_t& m){if(fail)return 0;tx.push_back(m);return 1;}}Keya_Bus;
struct SerialStub {template<class T>void print(T){}template<class T>void println(T){}void print(uint8_t,int){}void println(){}}Serial;
#define HEX 16
#define bitRead(x,n) (((x)>>(n))&1)
struct {float steerSensorCounts=30,AckermanFix=50;}steerSettings;
uint32_t lastKeyaHeatbeat=0;bool keyaDetected=true,keyaMotorFault=false,keyaTxFault=false,updateRawPositionOffset=false;
float intendedSteerAngle=0,keyaCurrentSteerAngleScaled=0,keyaCurrentSteeringPositionScaled=0,steerAngleSetPoint=3;int16_t KeyaCenterOffset=0,keyaCurrentActualSpeed=0,keyaCurrentSetSpeed=0;
int sensorReading=0,steerSwitch=1,currentState=1,previous=0;
void sendHardwareMessage(const char*,int){}
#include "zCANBUS.ino"
#include "SteeringSafety.h"
#include "AogPacket.h"
static bool commandIs(size_t i,const uint8_t* c){return std::memcmp(Keya_Bus.tx.at(i).buf,c,4)==0;}
int main(){
 CAN_Setup();assert(KeyaBusSendData.seq);Keya_Bus.tx.clear();
 SteerKeya(false);assert(commandIs(0,keyaDisableCommand));
 clockNow=10;SteerKeya(true);assert(commandIs(1,keyaSpeedCommand));
 clockNow=50;SteerKeya(false);assert(commandIs(2,keyaDisableCommand));
 clockNow=150;SteerKeya(false);assert(commandIs(3,keyaDisableCommand)); // no delayed enable
 clockNow=200;SteerKeya(true);assert(commandIs(4,keyaSpeedCommand));
 clockNow=299;SteerKeya(true);assert(Keya_Bus.tx.size()==5);
 clockNow=300;SteerKeya(true);assert(commandIs(5,keyaPositionCommand)&&commandIs(6,keyaEnableCommand));
 // Same target after disengagement MUST still be sent and enabled.
 clockNow=310;SteerKeya(false);assert(commandIs(7,keyaDisableCommand));
 clockNow=320;SteerKeya(true);assert(commandIs(8,keyaSpeedCommand));
 clockNow=420;SteerKeya(true);assert(commandIs(9,keyaPositionCommand)&&commandIs(10,keyaEnableCommand));
 // Calibration while active: caller revokes permission, disable precedes reset.
 updateRawPositionOffset=true;clockNow=430;SteerKeya(false);
 assert(commandIs(11,keyaDisableCommand)&&commandIs(12,keyaResetPosition));assert(!updateRawPositionOffset);
 // Heartbeat loss does not suppress transmission of disable.
 keyaDetected=false;clockNow=530;SteerKeya(false);assert(commandIs(13,keyaDisableCommand));
 // Failed disable is retried on next pass.
 Keya_Bus.fail=true;clockNow=630;SteerKeya(false);assert(Keya_Bus.tx.size()==14);
 Keya_Bus.fail=false;clockNow=631;SteerKeya(false);assert(commandIs(14,keyaDisableCommand));
 // Fault bits gate permission independently of AOG.
 CAN_message_t heartbeat;heartbeat.id=0x07000001;heartbeat.flags.extended=true;heartbeat.len=8;heartbeat.buf[7]=2;
 Keya_Bus.rx.push_back(heartbeat);KeyaBus_Receive();assert(keyaMotorFault);
 heartbeat.buf[7]=1;Keya_Bus.rx.push_back(heartbeat);KeyaBus_Receive();assert(!keyaMotorFault);
 heartbeat.len=1;heartbeat.buf[7]=2;Keya_Bus.rx.push_back(heartbeat);KeyaBus_Receive();assert(!keyaMotorFault);
 // Exercise permission and the real sender together, including a stop during
 // the 100 ms settling phase and recovery while AOG remains ON.
 SteeringSafety gate;
 auto run=[&](uint32_t t,float speed,bool aog){clockNow=t;gate.command(t,speed,aog);gate.update(t,keyaDetected&&!keyaMotorFault&&!keyaTxFault);SteerKeya(gate.motorAllowed);};
 keyaDetected=true;keyaMotorFault=false;keyaTxFault=false;
 run(1000,5,false);run(1010,5,true);size_t n=Keya_Bus.tx.size();
 assert(commandIs(n-1,keyaSpeedCommand));
 run(1050,0.9,true);assert(!gate.requested);assert(commandIs(Keya_Bus.tx.size()-1,keyaDisableCommand));
 for(uint32_t t=1100;t<1500;t+=10){run(t,5,true);assert(!gate.motorAllowed);}
 for(size_t i=n;i<Keya_Bus.tx.size();++i)assert(!commandIs(i,keyaEnableCommand));
 run(1500,5,false);run(1510,5,true);run(1610,5,true);assert(gate.motorAllowed);assert(commandIs(Keya_Bus.tx.size()-1,keyaEnableCommand));
 clockNow=2110;gate.update(clockNow,true);SteerKeya(gate.motorAllowed);assert(!gate.requested);assert(commandIs(Keya_Bus.tx.size()-1,keyaDisableCommand));
 // Enqueue failures latch a separate fault, which a clean heartbeat cannot clear.
 keyaDetected=true;keyaMotorFault=false;Keya_Bus.fail=true;clockNow=2200;SteerKeya(true);assert(keyaTxFault);
 heartbeat.len=8;heartbeat.buf[7]=1;Keya_Bus.rx.push_back(heartbeat);KeyaBus_Receive();assert(keyaTxFault);
 Keya_Bus.fail=false;clockNow=2210;SteerKeya(false);assert(!keyaTxFault);
 // Fractional PGN targets must survive the actual sender and retain
 // the upstream offset mapping (Ackermann 50 -> 4 degrees of slack).
 steerSettings.AckermanFix=50;steerSettings.steerSensorCounts=30;
 intendedSteerAngle=0;steerAngleSetPoint=decodeSteerDegrees(25,0);
 clockNow=2300;SteerKeya(true);clockNow=2400;SteerKeya(true);
 assert(std::fabs(intendedSteerAngle-3.75f)<0.0001f);
 assert(intendedAngleConverted==313); // 3.75 * 3 * 10000/360, rounded once
 auto packedPosition=[&](){for(auto i=Keya_Bus.tx.rbegin();i!=Keya_Bus.tx.rend();++i)if(std::memcmp(i->buf,keyaPositionCommand,4)==0)return int16_t((uint16_t(i->buf[4])<<8)|i->buf[5]);return int16_t(-9999);};
 assert(packedPosition()==313);
 steerAngleSetPoint=decodeSteerDegrees(50,0);clockNow=2410;SteerKeya(true);
 assert(std::fabs(intendedSteerAngle-3.5f)<0.0001f);assert(packedPosition()==292);
 steerAngleSetPoint=decodeSteerDegrees(0xE7,0xFF);clockNow=2420;SteerKeya(true); // -0.25
 assert(std::fabs(intendedSteerAngle-4.25f)<0.0001f);assert(packedPosition()==354);
 // Fractional feedback is no longer truncated by angle storage/functions.
 heartbeat.len=8;heartbeat.buf[0]=0;heartbeat.buf[1]=101;heartbeat.buf[7]=1;
 Keya_Bus.rx.push_back(heartbeat);KeyaBus_Receive();
 assert(std::fabs(keyaCurrentSteeringPositionScaled-(-101.f/3.f))<0.0001f);
 assert(std::fabs(keyaCurrentSteerAngleScaled-positionToDegrees(-101.f/3.f))<0.0001f);
 puts("PASS: fractional UDP decode, actual CAN target conversion and fractional feedback.");
 puts("PASS: real CAN sender: ordered commands, nonblocking wait, stop cancellation, identical-target re-engage, calibration priority, heartbeat loss, failed-send retry and fault decoding.");
}
