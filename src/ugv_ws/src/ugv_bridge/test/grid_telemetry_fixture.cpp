// 从真实C++解码器生成跨仓固定样本；这些数值只是软件测试输入。
#include "ugv_bridge/bx72_protocol.hpp"
#include <cstdint>
#include <iostream>

namespace {
void be(bx72::Bytes& b,std::uint64_t value,unsigned bytes) {
  for(unsigned i=0;i<bytes;++i)b.push_back(static_cast<std::uint8_t>(value>>((bytes-1-i)*8)));
}
bx72::Bytes header(unsigned kind,unsigned length) {
  bx72::Bytes b{'B','X','7','2',1,static_cast<std::uint8_t>(kind),0,static_cast<std::uint8_t>(length)};
  be(b,1,4);be(b,120000,4);return b;
}
}
int main() {
  auto nav=header(8,84);
  for (auto v : {1U,1U,7U,7U,1000000U,120000U}) be(nav,v,4);
  be(nav,123,8);nav.insert(nav.end(),{3,0,2,0xa6});
  be(nav,100,2);be(nav,0,2);be(nav,0,2);be(nav,7,2);
  for (auto v : {65536U,32768U,20U,10000U,1000U,42U}) be(nav,v,4);
  auto fusion=header(6,120);
  for (auto v : {250U,100U,0U,0U,0U,0U,0U,0U,0U}) be(fusion,v,4);
  for (auto v : {0x41f00000U,0x42200000U,0x42480000U,1024U,72U}) be(fusion,v,4);
  for (auto v : {0xd000c001U,2U,0U,500U,120000U,0U,0U,4U,1000U,1000U,0x00008303U,1U})
    be(fusion,v,4);
  auto plan=header(9,56);
  for (auto v : {1U,1U,7U,110000U,250U,100U,1000U,500U,9U,42U}) be(plan,v,4);
  const bx72::Json output={{"navigation",bx72::decode_telemetry(nav)},
                           {"fusion",bx72::decode_telemetry(fusion)},
                           {"local_plan",bx72::decode_telemetry(plan)}};
  std::cout<<output.dump(2)<<'\n';
}
