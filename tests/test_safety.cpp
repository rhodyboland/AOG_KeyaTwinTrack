#include "SteeringSafety.h"
#include "AogPacket.h"
#include <cassert>
#include <cstdio>
#include <vector>
#include <limits>
#include <cmath>
using S = SteeringSafety;
static void start(S& s, uint32_t t=0, float speed=5) {
 s.command(t,speed,false);s.update(t,true);
 s.command(t+10,speed,true);s.update(t+10,true);assert(s.motorAllowed);
}
int main() {
 {S s;s.update(0,true);assert(!s.motorAllowed);s.command(10,5,true);s.update(10,true);assert(!s.requested);}
 {S s;start(s);s.command(20,1.0,true);s.update(20,true);assert(s.motorAllowed);s.command(30,0.9,true);s.update(30,true);assert(!s.requested);s.command(40,5,true);s.update(40,true);assert(!s.requested);s.command(50,5,false);s.update(50,true);s.command(60,5,true);s.update(60,true);assert(s.motorAllowed);}
 {S s;s.command(0,1.4,false);s.update(0,true);s.command(10,1.4,true);s.update(10,true);assert(!s.requested);s.command(20,1.5,true);s.update(20,true);assert(!s.requested);s.command(30,1.5,false);s.update(30,true);s.command(40,1.5,true);s.update(40,true);assert(s.motorAllowed);}
 {S s;start(s);s.update(509,true);assert(s.motorAllowed);s.update(510,true);assert(!s.requested);s.command(520,5,true);s.update(520,true);assert(!s.requested);}
 {S s;start(s);s.update(20,false);assert(!s.requested);s.update(30,true);assert(!s.requested);s.command(40,5,true);s.update(40,true);assert(!s.requested);s.update(41,true,S::Engage);assert(s.motorAllowed);}
 {S s;start(s);s.command(20,5,false);s.update(20,true);assert(!s.requested);}
 {S s;s.command(0,5,false);s.update(0,true,S::Engage);assert(s.requested&&!s.motorAllowed);s.command(100,5,false);s.update(100,true);assert(s.requested);s.command(200,5,true);s.update(200,true);assert(s.motorAllowed);s.update(210,true,S::Disengage);assert(!s.requested);s.command(220,5,true);s.update(220,true);assert(!s.requested);}
 {S s;s.command(0,5,false);s.update(0,true,S::Engage);s.command(499,5,false);s.update(499,true);assert(s.requested);s.update(500,true);assert(!s.requested);s.command(510,5,true);s.update(510,true);assert(s.motorAllowed);/* OFF followed by a genuinely new ON after timeout. */}
 {S s;s.command(0,0,false);s.update(0,true,S::Engage);assert(!s.requested);s.command(10,5,false);s.update(10,true);assert(!s.requested);}
 {S s;start(s);s.command(20,25,true);s.update(20,true);assert(s.motorAllowed);s.command(30,25.1,true);s.update(30,true);assert(!s.requested);}
 for(float v:{-1.f,std::numeric_limits<float>::quiet_NaN(),std::numeric_limits<float>::infinity()}) {S s;start(s);s.command(20,v,true);s.update(20,true);assert(!s.requested);}
 {S s;start(s,0xFFFFFF00u);s.update(uint32_t(0xFFFFFF00u+509u),true);assert(s.motorAllowed);s.update(uint32_t(0xFFFFFF00u+510u),true);assert(!s.motorAllowed);}
 {S s;start(s);s.update(20,true,S::Disengage);s.command(30,5,true);s.update(30,true);assert(!s.requested);s.command(40,5,false);s.update(40,true);s.command(50,5,true);s.update(50,true,S::Disengage);assert(!s.requested);s.command(60,5,true);s.update(60,true);assert(!s.requested);}
 // AgOpenWeb PgnBuilder.BuildSteerConfigPgn emits five payload bytes:
 // button enable, threshold 8, min speed 1.5 kph, current sensor, angular velocity.
 // Keep this wire fixture independent of the packet helper below.
 {uint8_t config[]={0x80,0x81,0x7F,0xFB,5,0x40,8,15,4,0,0xDA};
  assert(validAogPacket(config,sizeof(config)));
  for(size_t n=0;n<sizeof(config);++n)assert(!validAogPacket(config,n));
  config[5]^=1;assert(!validAogPacket(config,sizeof(config)));config[5]^=1;
  config[4]=8;assert(!validAogPacket(config,sizeof(config)));}
 // Repeated ON packets cannot replay the edge after a fault.
 {S s;start(s);s.update(20,false);for(int i=30;i<10000;i+=10){s.command(i,5,true);s.update(i,true);assert(!s.requested);}}
 auto packet=[](uint8_t type, size_t n){std::vector<uint8_t> p(n);p[0]=0x80;p[1]=0x81;p[2]=0x7F;p[3]=type;p[4]=n-6;for(size_t i=2;i<n-1;++i)p[n-1]+=p[i];return p;};
 for(auto spec:{std::pair<int,int>{254,14},{252,14},{251,14},{251,11},{201,11},{202,9},{200,9}}){auto p=packet(spec.first,spec.second);assert(validAogPacket(p.data(),p.size()));p.back()^=1;assert(!validAogPacket(p.data(),p.size()));}
 {auto p=packet(254,14);for(size_t i=0;i<14;i++)assert(!validAogPacket(p.data(),i));p[5]^=1;assert(!validAogPacket(p.data(),p.size()));}
 {auto p=packet(254,13);assert(!validAogPacket(p.data(),p.size()));}
 for(size_t n:{size_t(9),size_t(10),size_t(12),size_t(13),size_t(15)}) {auto p=packet(251,n);assert(!validAogPacket(p.data(),p.size()));}
 for(int centi:{0,1,-1,25,-25,75,-75,99,-99,100,-100,32767,-32768}) {
   uint16_t bits=static_cast<uint16_t>(centi);
   assert(std::fabs(decodeSteerDegrees(bits&255,bits>>8)-centi*0.01f)<0.00001f);
 }
 puts("PASS: steering state transitions, thresholds, latching, handshake, timeout, wraparound, malformed packets and network-discovery packet compatibility.");
}
