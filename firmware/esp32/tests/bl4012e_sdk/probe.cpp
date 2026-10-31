
extern "C" {
#include "joint_bus.h"
}
#ifdef ADAPTER
#include "tap_hw/EspBlMotor.hpp"
#endif
#include <assert.h>
#include <initializer_list>
#include <stdio.h>
#include <string.h>
int GPIO, driven;
void event(const char* name,int value){printf("%s=%d\n",name,value);}
int64_t esp_timer_get_time(void){static int64_t tick;return ++tick;}
static int short_tx, short_echo;
static unsigned char incoming[128], response[64], sent[64];
static size_t used, at, response_len, sent_len;
static int corrupt_echo, tx_result;
int uart_flush_input(int p){(void)p;assert(!driven);event("flush",p);used=at=0;return 0;}
int uart_write_bytes(int p,const void *data,size_t n){
  assert(driven);event("write_uart",p);event("write_count",n);assert(n<=sizeof sent);memcpy(sent,data,n);sent_len=n;
  memcpy(incoming,data,n);if(corrupt_echo)incoming[0]^=1;
  memcpy(incoming+n,response,response_len);used=n+response_len;return short_tx?(int)n-1:(int)n;
}
int uart_wait_tx_done(int p,TickType_t t){assert(driven);event("wait_uart",p);event("wait_ticks",t);return tx_result;}
int uart_read_bytes(int p,void *data,size_t n,TickType_t t){
  assert(!driven);event("read_uart",p);event("read_count",n);event("read_ticks",t);if(short_echo && at==0) {short_echo=0;return 0;}if(n>used-at)n=used-at;memcpy(data,incoming+at,n);at+=n;return (int)n;
}
static void prepare(const unsigned char *bytes,size_t n){
  assert(n<=sizeof response);if(n)memcpy(response,bytes,n);response_len=n;
  corrupt_echo=tx_result=short_tx=short_echo=0;
}
static void same(const unsigned char *expected,size_t n){assert(sent_len==n&&!memcmp(sent,expected,n));for(size_t i=0;i<n;++i)event("byte",sent[i]);}
int main(void){
  joint_bus_t b;assert(joint_bus_init(&b,1,17,1000000)==ESP_OK);joint_reply_t r;
  // Manufacturer HTML section 7.1: checksum in its TX example is wrong.
  const unsigned char mit_rx[]={0xff,0xff,1,7,0,0,0x10,0x0d,3,0x1a,0xbd};
  const unsigned char mit_tx[]={0xff,0xff,1,0x0d,3,0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0,0x69};
  joint_mit_cmd_t cmd={10000,36,4096,0,0};prepare(mit_rx,sizeof mit_rx);
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_OK);same(mit_tx,sizeof mit_tx);
  assert(r.pos==4096&&r.status==0);
  // Section 7.2: position-only write, correct documented checksum and position.
  const unsigned char pos_rx[]={0xff,0xff,1,7,0,0x40,0x1f,5,3,0x1e,0x72};
  const unsigned char pos_tx[]={0xff,0xff,1,5,3,0x1e,0,0x20,0xb8};
  const unsigned char position[]={0,0x20};prepare(pos_rx,sizeof pos_rx);
  assert(bus_write(&b,JB_FMT_JOINT,1,0x1e,position,2,&r)==ESP_OK);
  same(pos_tx,sizeof pos_tx);assert(r.pos==8000);
  // Wheel format has no embedded joint position before the echoed instruction.
  const unsigned char wheel_rx[]={0xff,0xff,1,0x0b,0,4,2,0x24,0,2,0xc8,0,0xf6,0xff,0x0a};
  const unsigned char wheel_tx[]={0xff,0xff,1,4,2,0x24,6,0xce};
  unsigned char out[48];prepare(wheel_rx,sizeof wheel_rx);
  assert(bus_read(&b,JB_FMT_WHEEL,1,0x24,6,out,&r)==ESP_OK);same(wheel_tx,sizeof wheel_tx);
  assert(out[0]==0&&out[1]==2&&out[2]==0xc8&&out[3]==0&&out[4]==0xf6&&out[5]==0xff);
  // Timeout / bad echo / bad checksum are distinct errors, not successful apply.
  prepare(NULL,0);assert(bus_ping(&b,JB_FMT_JOINT,1,&r)==ESP_ERR_TIMEOUT);assert(b.err_timeout==1);
  prepare(mit_rx,sizeof mit_rx);corrupt_echo=1;
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_INVALID_RESPONSE);assert(b.err_echo==1);
  prepare(mit_rx,sizeof mit_rx);response[response_len-1]^=1;
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_INVALID_RESPONSE);assert(b.err_frame==1);
  prepare(mit_rx,sizeof mit_rx);tx_result=ESP_ERR_TIMEOUT;
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_ERR_TIMEOUT);assert(b.err_timeout==2);
  // A broadcast has TX echo, but no motor reply. Oversized reads are rejected.
  const unsigned char off=0;prepare(NULL,0);
  assert(bus_write(&b,JB_FMT_JOINT,0xfe,0x18,&off,1,&r)==ESP_OK);
  assert(bus_read(&b,JB_FMT_JOINT,0xfe,0x24,2,out,&r)==ESP_ERR_INVALID_ARG);
  assert(bus_read(&b,JB_FMT_JOINT,1,0x24,sizeof out,out,&r)==ESP_ERR_INVALID_SIZE);
  puts("PASS: actual joint_bus.c; MIT/position/wheel frames, echo, reply offset, checksum, timeout and broadcast");

#ifdef CANDIDATE
  prepare(mit_rx,sizeof mit_rx);short_tx=1;
  assert(bus_write_mit(&b,1,&cmd,&r)!=ESP_OK && b.last_error==JB_ERROR_TX && !driven);
  prepare(mit_rx,sizeof mit_rx);short_echo=1;
  assert(bus_write_mit(&b,1,&cmd,&r)!=ESP_OK && b.last_error==JB_ERROR_ECHO_TIMEOUT && !driven);
  prepare(mit_rx,sizeof mit_rx);corrupt_echo=1;
  assert(bus_write_mit(&b,1,&cmd,&r)!=ESP_OK && b.last_error==JB_ERROR_ECHO_MISMATCH);
  prepare(NULL,0);
  assert(bus_write_mit(&b,1,&cmd,&r)!=ESP_OK && b.last_error==JB_ERROR_REPLY_TIMEOUT);
  assert(b.trace.tx_count==17 && b.trace.echo_count==17 && b.trace.rx_count==0 && b.trace.expected==11);
  for(unsigned field: {0u,2u,3u,7u,8u,9u,10u}) {
    prepare(mit_rx,sizeof mit_rx);response[field]^=1;
    // Preserve checksum except when deliberately corrupting it.
    if(field!=10){unsigned sum=0;for(unsigned j=2;j<10;++j)sum+=response[j];response[10]=(unsigned char)~sum;}
    assert(bus_write_mit(&b,1,&cmd,&r)!=ESP_OK && b.last_error==JB_ERROR_REPLY_INVALID);
  }
  prepare(mit_rx,sizeof mit_rx);
  assert(bus_write_mit(&b,1,&cmd,&r)==ESP_OK && b.last_error==JB_ERROR_NONE);
  assert(b.trace.rx_count==11 && !memcmp(b.trace.rx,mit_rx,11));
#endif
#ifdef ADAPTER
  using namespace tap_hw::hybrid;
  EspMotorBus adapter;BusReply decoded{};
  const unsigned char read_params[]={0x24,6};
  assert(!adapter.request(BusFormat::Wheel,1,2,read_params,2,6,decoded));
  assert(adapter.lastError()==BusError::Unavailable);
  assert(adapter.begin(1,17,18));
  prepare(wheel_rx,sizeof wheel_rx);
  assert(adapter.request(BusFormat::Wheel,1,2,read_params,2,6,decoded));
  same(wheel_tx,sizeof wheel_tx);assert(decoded.data[4]==0xf6 && decoded.data[5]==0xff);
  assert(adapter.trace().expected==15 && adapter.trace().rx_count==15);
  const unsigned char mit_params[]={0x1a,0x10,0x27,0x24,0,0,0x10,0,0,0,0};
  prepare(mit_rx,sizeof mit_rx);
  assert(adapter.request(BusFormat::Joint,1,3,mit_params,sizeof mit_params,0,decoded));
  same(mit_tx,sizeof mit_tx);assert(decoded.position==4096);
  for(int fault=0;fault<6;++fault){
    prepare(mit_rx,sizeof mit_rx);
    BusError expected;
    switch(fault){
      case 0:tx_result=ESP_ERR_TIMEOUT;expected=BusError::Tx;break;
      case 1:short_tx=1;expected=BusError::Tx;break;
      case 2:short_echo=1;expected=BusError::EchoTimeout;break;
      case 3:corrupt_echo=1;expected=BusError::EchoMismatch;break;
      case 4:response_len=0;expected=BusError::ReplyTimeout;break;
      default:response[10]^=1;expected=BusError::ReplyInvalid;break;
    }
    decoded.position=1234;
    assert(!adapter.request(BusFormat::Joint,1,3,mit_params,sizeof mit_params,0,decoded));
    assert(adapter.lastError()==expected && decoded.position==1234 && !driven);
  }
  for(auto id:{0,254,255}){
    assert(!adapter.request(BusFormat::Joint,id,1,nullptr,0,0,decoded));
    assert(adapter.lastError()==BusError::Packet);
  }
  assert(!adapter.request(BusFormat::Wheel,1,2,read_params,2,2,decoded));
  assert(!adapter.request(BusFormat::Wheel,1,2,nullptr,2,6,decoded));
  assert(!adapter.request(BusFormat::Joint,1,3,mit_params,43,0,decoded));
  assert(!adapter.request(BusFormat::Joint,1,9,nullptr,0,0,decoded));
  // Actual candidate frame counts for one active batch: 2 MIT + speed + present6.
  prepare(mit_rx,sizeof mit_rx);
  assert(adapter.request(BusFormat::Joint,1,3,mit_params,sizeof mit_params,0,decoded));
  unsigned bytes=2*(adapter.trace().tx_count+adapter.trace().rx_count);
  const unsigned char speed_rx[]={0xff,0xff,1,5,0,5,3,0x20,0xd1};
  const unsigned char speed_params[]={0x20,0,0};prepare(speed_rx,sizeof speed_rx);
  assert(adapter.request(BusFormat::Wheel,1,3,speed_params,3,0,decoded));
  bytes+=adapter.trace().tx_count+adapter.trace().rx_count;
  prepare(wheel_rx,sizeof wheel_rx);
  assert(adapter.request(BusFormat::Wheel,1,2,read_params,2,6,decoded));
  bytes+=adapter.trace().tx_count+adapter.trace().rx_count;
  assert(bytes==97);
#endif
  return 0;
}
