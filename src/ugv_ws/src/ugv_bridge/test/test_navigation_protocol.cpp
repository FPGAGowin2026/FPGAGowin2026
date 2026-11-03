#include <gtest/gtest.h>
#include <algorithm>
#include <stdexcept>
#include "ugv_bridge/bx72_protocol.hpp"

namespace {
bx72::NavigationTask sample() {
  bx72::NavigationTask t;
  t.task_id=23;t.odom_domain=9;t.boot_id=0x12345678;
  t.goal_x_q16=-32768;t.goal_y_q16=65536;t.hold_yaw_q16=-102944;t.final_yaw_q16=102944;
  t.sent_at_us=0x100ULL;t.expiry_us=t.sent_at_us+30000000;t.flags=1;
  return t;
}
void be(bx72::Bytes& b,std::uint64_t v,unsigned n) {
  for(unsigned i=0;i<n;++i)b.push_back(static_cast<std::uint8_t>(v>>((n-1-i)*8)));
}
bx72::Json telemetry(const char* type,std::uint32_t sequence,std::uint32_t time_us,
                     std::uint32_t boot_id=0) {
  bx72::Json value={{"type",type},{"sequence",sequence},{"time_us",time_us}};
  if (std::string(type)=="navigation" || std::string(type)=="local_plan" ||
      std::string(type)=="local_grid") value["boot_id"]=boot_id;
  return value;
}
bx72::Bytes grid_chunk(unsigned part,std::uint32_t id=9,std::uint32_t boot=7,
                       std::uint32_t begin=1000000,std::uint32_t end=1000800) {
  bx72::Bytes bytes{'B','X','7','2',1,10};
  be(bytes,224,2);be(bytes,id*8+part,4);be(bytes,end+part*10,4);
  const std::array<std::uint32_t,20> header{boot,1,id,begin,40,12,
      static_cast<std::uint32_t>(-33),static_cast<std::uint32_t>(-28),
      begin-10000,begin-1000,40,static_cast<std::uint32_t>(-7),100,200,
      static_cast<std::uint32_t>(-1571),2,3,7,end,0x01320800U|part};
  for (const auto value:header)be(bytes,value,4);
  const std::array<std::uint8_t,3> values{0x24,0x55,0xaa};
  for(unsigned i=0;i<128;++i)bytes.push_back(values[(i+part)%3]);
  return bytes;
}
}
namespace {
bx72::Bytes global_map_chunk() {
  bx72::Bytes bytes{'B','X','7','2',1,8,0,160};
  be(bytes,0x12345678U,4); be(bytes,0x10203040U,4);
  // 分片元数据为小端；外层遥测序号和时间仍为大端。
  bytes.insert(bytes.end(),{0x4d,0x02, 0x34,0x12, 0xa5,0x01, 0x00,0x20,
                            10,10,64,0x03, 0xff,0x03, 0x00,0x02});
  for (unsigned i=0;i<128;++i) bytes.push_back(static_cast<std::uint8_t>(i%64));
  return bytes;
}
}
TEST(GlobalMapTelemetry, DecodesAndEncapsulatesOneChunkWithoutAssemblingSnapshot) {
  const auto bytes=global_map_chunk();
  ASSERT_EQ(bytes.size(),160U);
  const auto decoded=bx72::decode_telemetry(bytes);
  EXPECT_EQ(decoded.at("type"),"map");
  EXPECT_EQ(decoded.at("protocol"),"occupancy_global_v1");
  EXPECT_EQ(decoded.at("frame_id"),"map");
  EXPECT_EQ(decoded.at("source"),"fpga");
  EXPECT_EQ(decoded.at("sequence"),0x12345678U);
  EXPECT_EQ(decoded.at("time_us"),0x10203040U);
  EXPECT_EQ(decoded.at("snapshot_sequence"),0x1234U);
  EXPECT_EQ(decoded.at("chunk_index"),0x01a5U);
  EXPECT_EQ(decoded.at("chunk_count"),8192);
  EXPECT_EQ(decoded.at("width"),1024); EXPECT_EQ(decoded.at("height"),1024);
  EXPECT_EQ(decoded.at("resolution_mm"),64); EXPECT_EQ(decoded.at("flags"),3);
  EXPECT_EQ(decoded.at("robot_cell_x"),1023); EXPECT_EQ(decoded.at("robot_cell_y"),512);
  ASSERT_EQ(decoded.at("cells").size(),128U);
  for (unsigned i=0;i<128;++i) EXPECT_EQ(decoded.at("cells")[i],i%64);
  const auto envelopes=bx72::telemetry_envelopes("map-test",7,decoded,1,1.01);
  ASSERT_EQ(envelopes.size(),1U);
  EXPECT_EQ(envelopes[0].at("stream"),"map");
  EXPECT_EQ(envelopes[0].at("data"),decoded);
  EXPECT_LE(envelopes[0].dump().size(),1400U);
}
TEST(GlobalMapTelemetry, RejectsMalformedMetadataLengthAndReservedCellBits) {
  const auto valid=global_map_chunk();
  for (const auto& [offset,value] : std::array<std::pair<unsigned,std::uint8_t>,10>{{
      {16,0}, {17,0}, {22,1}, {24,9}, {25,9}, {26,63}, {27,0},
      {29,4}, {31,4}, {32,0x40}}}) {
    auto invalid=valid; invalid[offset]=value;
    EXPECT_THROW(bx72::decode_telemetry(invalid),std::invalid_argument)<<offset;
  }
  auto index_outside=valid; index_outside[20]=0; index_outside[21]=0x20;
  EXPECT_THROW(bx72::decode_telemetry(index_outside),std::invalid_argument);
  auto wrong_length=valid; wrong_length[7]=159;
  EXPECT_THROW(bx72::decode_telemetry(wrong_length),std::invalid_argument);
  auto truncated=valid; truncated.pop_back(); truncated[7]=159;
  EXPECT_THROW(bx72::decode_telemetry(truncated),std::invalid_argument);
}
TEST(GlobalMapTelemetry, Kind8NavigationAndKind10GridRemainIndependent) {
  bx72::Bytes navigation(84,0);
  navigation[0]='B'; navigation[1]='X'; navigation[2]='7'; navigation[3]='2';
  navigation[4]=1; navigation[5]=8; navigation[7]=84; navigation[51]=0xa0;
  const auto nav=bx72::decode_telemetry(navigation);
  EXPECT_EQ(nav.at("type"),"navigation"); EXPECT_EQ(nav.at("protocol"),"nav_grid_v1");
  const auto grid=bx72::decode_telemetry(grid_chunk(0));
  EXPECT_EQ(grid.at("type"),"local_grid_chunk");
  EXPECT_EQ(grid.at("protocol"),"nav_grid_chunk_v1");
  EXPECT_EQ(bx72::decode_telemetry(global_map_chunk()).at("type"),"map");
}
TEST(LocalGridProtocol, DecodesSignedCoordinatesPackingAndRejectsInvalidWireValues) {
  const auto bytes=grid_chunk(3);
  const auto chunk=bx72::decode_telemetry(bytes);
  EXPECT_EQ(chunk["type"],"local_grid_chunk");
  EXPECT_EQ(chunk["origin_x_cell"],-33);EXPECT_EQ(chunk["origin_y_cell"],-28);
  EXPECT_EQ(chunk["grid_from_odom"]["yaw_offset_mrad"],-1571);
  EXPECT_EQ(chunk["chunk_index"],3);EXPECT_EQ(chunk["chunk_cells"].size(),128U);
  EXPECT_EQ(chunk["chunk_cells"][0],bytes[96]);
  for (const auto index:{5U,6U,84U,92U,95U,96U}) {
    auto invalid=bytes;
    invalid[index]=index==96 ? 255 : index==95 ? 8 : 255;
    EXPECT_THROW(bx72::decode_telemetry(invalid),std::invalid_argument)<<index;
  }
}
TEST(LocalGridProtocol, ReassemblesOneFrozenSnapshotInPartOrder) {
  bx72::LocalGridAssembler assembler;
  assembler.set_domain(7,1);
  std::optional<bx72::Json> result;
  const std::array<unsigned,8> order{7,0,3,1,5,2,4,6};
  for(unsigned i=0;i<order.size();++i) {
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(order[i])),i*.01);
    EXPECT_EQ(result.has_value(),i==7);
  }
  ASSERT_TRUE(result);
  EXPECT_EQ((*result)["protocol"],"nav_grid_snapshot_v1");
  EXPECT_EQ((*result)["type"],"local_grid");EXPECT_EQ((*result)["sequence"],9);
  EXPECT_EQ((*result)["time_us"],1000800);
  bx72::Bytes expected;
  for(unsigned part=0;part<8;++part) {
    const auto wire=grid_chunk(part);
    expected.insert(expected.end(),wire.begin()+96,wire.end());
  }
  EXPECT_EQ((*result)["packed_cells"].get<bx72::Bytes>(),expected);
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0)),.1));
  const auto envelopes=bx72::telemetry_envelopes("grid-test",1,*result,.1,.1,1400);
  EXPECT_GT(envelopes.size(),1U);EXPECT_LE(envelopes.size(),32U);
  bx72::Bytes forwarded;
  for(const auto& envelope:envelopes) {
    EXPECT_LE(envelope.dump().size(),1400U);
    if(envelope["data"].contains("packed_cells")) {
      const auto part=envelope["data"]["packed_cells"].get<bx72::Bytes>();
      forwarded.insert(forwarded.end(),part.begin(),part.end());
    }
  }
  EXPECT_EQ(forwarded,expected);
}
TEST(LocalGridProtocol, MissingDuplicateConflictingAndMixedMetadataNeverPublish) {
  bx72::LocalGridAssembler assembler;
  assembler.set_domain(7,1);
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0)),0));
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0)),.24));
  for(unsigned part=1;part<8;++part)
    EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(part)),.251));
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0,10)),.3));
  auto conflict=grid_chunk(0,10);conflict[96]=0x55;
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(conflict),.31));
  for(unsigned part=1;part<8;++part)
    EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(part,10)),.32));
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0,11)),.4));
  auto mismatch=grid_chunk(1,11);mismatch[35]++;
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(mismatch),.41));
  for(unsigned part=1;part<8;++part)
    EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(part,11)),.42));
  std::optional<bx72::Json> result;
  for(unsigned part=0;part<8;++part)
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(part,12)),.5);
  ASSERT_TRUE(result);EXPECT_EQ((*result)["snapshot_id"],12);
}
TEST(LocalGridProtocol, HandlesCounterWrapAndRetiresPreviousBoot) {
  bx72::LocalGridAssembler assembler;
  assembler.set_domain(7,1);
  std::optional<bx72::Json> result;
  for(unsigned part=0;part<8;++part)
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(part,0xffffffffU,7,0xffffffe0U,16)),0);
  ASSERT_TRUE(result);
  for(unsigned part=0;part<8;++part)
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(part,0,7,32,64)),.1);
  ASSERT_TRUE(result);EXPECT_EQ((*result)["snapshot_id"],0);
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0,0xfffffffeU)),.11));
  assembler.set_domain(8,1);
  for(unsigned part=0;part<8;++part)
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(part,1,8)),.2);
  ASSERT_TRUE(result);EXPECT_EQ((*result)["boot_id"],8);
  for(unsigned part=0;part<8;++part)
    EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(part,2,7)),.21));
}
TEST(LocalGridProtocol, ForeignFragmentCannotEvictConfirmedNavigationDomain) {
  bx72::LocalGridAssembler assembler;
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0)),0));
  assembler.set_domain(7,1);
  std::optional<bx72::Json> result;
  for(unsigned part=0;part<8;++part)
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(part)),.01);
  ASSERT_TRUE(result);
  EXPECT_FALSE(assembler.feed(bx72::decode_telemetry(grid_chunk(0,1,6)),.02));
  for(unsigned part=0;part<8;++part)
    result=assembler.feed(bx72::decode_telemetry(grid_chunk(part,10)),.03);
  ASSERT_TRUE(result);EXPECT_EQ((*result)["boot_id"],7);
}
TEST(ControlProtocol, SignedManualYawLimitsUseNetworkByteOrder) {
  for (const int yaw : {800, -800}) {
    const auto packet=bx72::fpga_packet(1,2,{0,0,yaw});
    ASSERT_EQ(packet.size(),32U);
    EXPECT_EQ(packet[5],3);
    EXPECT_EQ(packet[24],yaw>0 ? 0x03 : 0xfc);
    EXPECT_EQ(packet[25],yaw>0 ? 0x20 : 0xe0);
    EXPECT_EQ((std::uint16_t(packet[30])<<8)|packet[31],bx72::crc16(packet,30));
  }
}
TEST(ControlProtocol, AcknowledgesOnlySupportedCommands) {
  for (auto request : {bx72::fpga_packet(1,2,{10,0,0}), bx72::fpga_auto_packet(2,2,false)}) {
    auto ack=request; ++ack[5];
    auto crc=bx72::crc16(ack,30);ack[30]=crc>>8;ack[31]=crc;
    EXPECT_TRUE(bx72::valid_ack(ack,request));
    ack[19]^=1; crc=bx72::crc16(ack,30);ack[30]=crc>>8;ack[31]=crc;
    EXPECT_FALSE(bx72::valid_ack(ack,request));
  }
  auto request=bx72::fpga_packet(3,2,{}); request[5]=9;
  auto ack=request; ack[5]=10;
  const auto crc=bx72::crc16(ack,30);ack[30]=crc>>8;ack[31]=crc;
  EXPECT_FALSE(bx72::valid_ack(ack,request));
}
TEST(NavigationProtocol, FragmentTransportPreservesSignedOdomAndWrapsSequence) {
  const auto task=sample();
  const auto encoded=bx72::encode_navigation_task(task);
  const auto packets=bx72::fpga_task_packets(0xfffffffcU,0x123456789abcdef0ULL,task);
  ASSERT_EQ(encoded.size(),72U);
  ASSERT_EQ(packets.size(),8U);
  bx72::Bytes restored;
  for (unsigned i=0;i<8;++i) {
    const auto& packet=packets[i];
    ASSERT_EQ(packet.size(),32U);
    EXPECT_EQ(packet[5],7); EXPECT_EQ(packet[20],i);
    const auto seq=(std::uint32_t(packet[8])<<24)|(std::uint32_t(packet[9])<<16)|
                   (std::uint32_t(packet[10])<<8)|packet[11];
    EXPECT_EQ(seq,0xfffffffcU+i);
    restored.insert(restored.end(),packet.begin()+21,packet.begin()+30);
    auto ack=packet; ack[5]=8;
    auto crc=bx72::crc16(ack,30); ack[30]=crc>>8; ack[31]=crc;
    EXPECT_TRUE(bx72::valid_ack(ack,packet));
    ack[29]^=1; crc=bx72::crc16(ack,30); ack[30]=crc>>8; ack[31]=crc;
    EXPECT_FALSE(bx72::valid_ack(ack,packet));
  }
  EXPECT_EQ(restored,encoded);
  const auto decoded=bx72::decode_navigation_task(restored);
  EXPECT_EQ(decoded.goal_x_q16,task.goal_x_q16);
  EXPECT_EQ(decoded.goal_y_q16,task.goal_y_q16);
  EXPECT_EQ(decoded.odom_domain,task.odom_domain);
  EXPECT_EQ(decoded.expiry_us,task.expiry_us);
}
TEST(NavigationProtocol, DefaultGoalToleranceMatchesPolarNavigationThreshold) {
  const bx72::NavigationTask task;
  EXPECT_EQ(task.tolerance_mm,100);
}
TEST(NavigationProtocol, DefaultLinearSpeedIs400MillimetersPerSecond) {
  EXPECT_EQ(sample().max_speed_mm_s,400);
}
TEST(NavigationProtocol, RuntimeDomainTimeAndTaskSequence) {
  auto t=sample();const auto now=t.sent_at_us+500000;
  EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id,22),0);
  EXPECT_EQ(bx72::navigation_task_error(t,now+1,t.odom_domain,t.boot_id,22),2);
  EXPECT_EQ(bx72::navigation_task_error(t,t.sent_at_us-1,t.odom_domain,t.boot_id),2);
  EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain+1,t.boot_id),1);
  EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id+1),1);
  EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id,23),3);
  EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id,24),3);
  t.task_id=1;EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id,0xffffffffU),0);
  t.task_id=0x80000001U;EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id,1),3);
  t=sample();t.action=3;EXPECT_EQ(bx72::navigation_task_error(t,now,t.odom_domain,t.boot_id,23),3);
  t=sample();t.sent_at_us=0xfffffff0U;t.expiry_us=0x10U;
  EXPECT_EQ(bx72::navigation_task_error(t,0xfffffff8U,t.odom_domain,t.boot_id,22),0);
  EXPECT_EQ(bx72::navigation_task_error(t,0x10U,t.odom_domain,t.boot_id,22),2);
}
TEST(TelemetryTracker, ConfirmsRestartAcrossHalfRangeAndRejectsRetiredEpochPackets) {
  for (const std::uint32_t old_time : {1500000000U,2208012904U})
   for (const std::uint32_t new_time : {0U,1000000U,5000000U}) {
    SCOPED_TRACE(old_time);
    SCOPED_TRACE(new_time);
    bx72::TelemetryTracker tracker;
    bool restarted=false;
    ASSERT_TRUE(tracker.observe(telemetry("navigation",110399,old_time,7),10.0));
    ASSERT_TRUE(tracker.observe(telemetry("motor",110399,old_time),10.001));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",1,new_time,8),12.0,&restarted));
    EXPECT_FALSE(restarted);
    EXPECT_FALSE(tracker.observe(telemetry("motor",1,new_time+5000),12.005,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",2,new_time+20000,8),12.020,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("motor",2,new_time+25000),12.025,&restarted));
    EXPECT_TRUE(restarted);
    EXPECT_EQ(tracker.epoch,2U);
    EXPECT_EQ(tracker.streams.at("navigation").sequence,2U);
    EXPECT_EQ(tracker.streams.at("motor").sequence,2U);
    // 旧序号比新序号大，两个半范围场景都不能污染新 epoch。
    EXPECT_FALSE(tracker.observe(telemetry("navigation",110398,old_time-20000,7),14.0,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",110398,old_time-20000),14.001,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("health",110398,old_time-20000),14.002,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",110398,new_time+2000000,7),14.003,&restarted));
    EXPECT_FALSE(restarted);
    EXPECT_EQ(tracker.streams.at("navigation").sequence,2U);
    EXPECT_EQ(tracker.streams.at("motor").sequence,2U);
    EXPECT_TRUE(tracker.observe(telemetry("navigation",3,new_time+2025000,8),14.025,&restarted));
    EXPECT_FALSE(restarted);
    EXPECT_EQ(tracker.epoch,2U);
    // 同时到达的批量样本允许合理的源时间差。
    EXPECT_TRUE(tracker.observe(telemetry("navigation",4,new_time+2525000,8),14.025,&restarted));
    // 以后长时间静默、正常时间前进及自然回绕仍可接受。
    EXPECT_TRUE(tracker.observe(telemetry("navigation",5,new_time+2000025000U,8),2012.025,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("navigation",6,new_time+4000025000U,8),4012.025,&restarted));
    const auto wrapped=static_cast<std::uint32_t>(std::uint64_t(new_time)+4300025000ULL);
    EXPECT_TRUE(tracker.observe(telemetry("navigation",7,wrapped,8),4312.025,&restarted));
    EXPECT_FALSE(restarted);
    EXPECT_EQ(tracker.epoch,2U);
  }
}
TEST(TelemetryTracker, ConfirmsDelayedRestartAfterClockWrapWithChangedBoot) {
  constexpr std::uint32_t old_time=4072664482U;
  constexpr std::uint32_t old_boot=3375280997U;
  constexpr std::uint32_t new_boot=2598136569U;
  for (const auto& scenario : {std::pair<std::uint32_t,double>{60000001U,400.0},
                               {100000000U,400.0}, {380125002U,720.0},
                               {500000000U,900.0}}) {
    const auto new_time=scenario.first;
    const auto resumed=10.0+scenario.second;
    SCOPED_TRACE(new_time);
    bx72::TelemetryTracker tracker;
    bool restarted=false;
    ASSERT_TRUE(tracker.observe(telemetry("navigation",203632,old_time,old_boot),10.0));
    ASSERT_TRUE(tracker.observe(telemetry("motor",203632,old_time),10.0));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",19005,new_time,new_boot),resumed,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",19005,new_time+5000),resumed+.005,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",19006,new_time+20000,new_boot),resumed+.020,&restarted));
    EXPECT_FALSE(restarted);
    EXPECT_EQ(tracker.epoch,1U);
    // A mismatched stream must not count or poison the pending motor vote.
    EXPECT_FALSE(tracker.observe(telemetry("motor",19006,new_time+4000000),resumed+.021,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",19007,new_time+25000),resumed+.025,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("motor",19008,new_time+45000),resumed+.045,&restarted));
    EXPECT_TRUE(restarted);
    EXPECT_EQ(tracker.epoch,2U);
    ASSERT_EQ(tracker.streams.at("navigation").boot_id,new_boot);
    EXPECT_FALSE(tracker.observe(telemetry("navigation",203631,old_time-20000,old_boot),resumed+.050,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",203631,old_time-20000),resumed+.051,&restarted));
    EXPECT_FALSE(restarted);
    EXPECT_EQ(tracker.epoch,2U);
    EXPECT_TRUE(tracker.observe(telemetry("navigation",19007,new_time+60000,new_boot),resumed+.060,&restarted));
  }
}
TEST(TelemetryTracker, LateClockWrapNeedsChangedBootAndIndependentStreams) {
  constexpr std::uint32_t old_time=4072664482U;
  constexpr std::uint32_t old_boot=3375280997U;
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",203632,old_time,old_boot),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",203632,old_time),10.0));
  for (std::uint32_t i=0;i<3;++i) {
    EXPECT_FALSE(tracker.observe(telemetry("navigation",i+1,100000000U+i*20000,old_boot),
                                 410.0+i*.020,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",i+1,100000000U+i*20000),
                                 410.0+i*.020,&restarted));
  }
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
  // A new navigation boot alone is also insufficient to advance the epoch.
  for (std::uint32_t i=0;i<3;++i)
    EXPECT_FALSE(tracker.observe(telemetry("navigation",i+1,110000000U+i*20000,8),
                                 420.0+i*.020,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);

  // Navigation and local_plan are the same group, even for a changed boot.
  bx72::TelemetryTracker related;
  ASSERT_TRUE(related.observe(telemetry("navigation",203632,old_time,old_boot),10.0));
  ASSERT_TRUE(related.observe(telemetry("local_plan",203632,old_time,old_boot),10.0));
  EXPECT_FALSE(related.observe(telemetry("navigation",1,100000000U,8),410.0,&restarted));
  EXPECT_FALSE(related.observe(telemetry("navigation",2,100020000U,8),410.020,&restarted));
  EXPECT_FALSE(related.observe(telemetry("local_plan",1,100025000U,8),410.025,&restarted));
  EXPECT_FALSE(related.observe(telemetry("local_plan",2,100045000U,8),410.045,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(related.epoch,1U);
}
TEST(TelemetryTracker, LateClockWrapWithoutNavigationBootStaysUnconfirmed) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("imu",203632,4072664482U),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",203632,4072664482U),10.0));
  EXPECT_FALSE(tracker.observe(telemetry("imu",1,100000000U),110.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,100005000U),110.005,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("imu",2,100020000U),110.020,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",2,100025000U),110.025,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
}
TEST(TelemetryTracker, RejectsReplayOfBootRetiredByForwardNavigation) {
  constexpr std::uint32_t old_boot=7, current_boot=9;
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",19003,380085002U,old_boot),1.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",19003,380085002U),1.0));
  ASSERT_TRUE(tracker.observe(telemetry("navigation",114003,2280085002U,current_boot),1901.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",114003,2280085002U),1901.0));
  ASSERT_TRUE(tracker.observe(telemetry("navigation",203632,4072664482U,current_boot),3693.57948));
  ASSERT_TRUE(tracker.observe(telemetry("motor",203632,4072664482U),3693.57948));

  EXPECT_FALSE(tracker.observe(telemetry("navigation",19005,380125002U,old_boot),4093.57948,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",19006,380145002U,old_boot),4093.59948,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",19005,380125002U),4093.60448,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",19006,380145002U),4093.62448,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
  EXPECT_EQ(tracker.streams.at("navigation").boot_id,current_boot);
  EXPECT_NE(std::find(tracker.retired_boot_ids.begin(),tracker.retired_boot_ids.end(),old_boot),
            tracker.retired_boot_ids.end());
  EXPECT_EQ(std::find(tracker.retired_boot_ids.begin(),tracker.retired_boot_ids.end(),current_boot),
            tracker.retired_boot_ids.end());
  EXPECT_TRUE(tracker.observe(telemetry("navigation",223635,177757186U,current_boot),4093.63948,&restarted));
  EXPECT_FALSE(restarted);
}
TEST(TelemetryTracker, KeepsTrustedBootAcrossForwardRejectedNavigation) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",203632,4072664482U,7),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",203632,4072664482U),10.0));
  auto rejected=telemetry("navigation",203633,4072684482U,9);
  rejected["rejected"]=true;
  ASSERT_TRUE(tracker.observe(rejected,10.02,&restarted));
  EXPECT_TRUE(tracker.streams.at("navigation").boot_known);
  EXPECT_EQ(tracker.streams.at("navigation").boot_id,7U);
  EXPECT_TRUE(tracker.retired_boot_ids.empty());

  EXPECT_FALSE(tracker.observe(telemetry("navigation",19005,380125002U,9),730.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",19006,380145002U,9),730.020,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",19005,380150002U),730.025,&restarted));
  EXPECT_TRUE(tracker.observe(telemetry("motor",19006,380170002U),730.045,&restarted));
  EXPECT_TRUE(restarted);
  EXPECT_EQ(tracker.epoch,2U);
  EXPECT_EQ(tracker.streams.at("navigation").boot_id,9U);
}
TEST(TelemetryTracker, RejectedNavigationBootCannotStartRestartVote) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",203632,4072664482U,7),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",203632,4072664482U),10.0));
  auto rejected=telemetry("navigation",19005,380125002U,9);
  rejected["rejected"]=true;
  EXPECT_FALSE(tracker.observe(rejected,730.0,&restarted));
  rejected["sequence"]=19006; rejected["time_us"]=380145002U;
  EXPECT_FALSE(tracker.observe(rejected,730.020,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",19005,380150002U),730.025,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",19006,380170002U),730.045,&restarted));
  EXPECT_EQ(tracker.reset_votes.find("navigation"),tracker.reset_votes.end());
  EXPECT_EQ(tracker.epoch,1U);
  EXPECT_FALSE(restarted);

  // A rejection cannot establish a boot identity when none was trusted yet.
  bx72::TelemetryTracker uninitialized;
  rejected["sequence"]=1; rejected["time_us"]=1000000;
  ASSERT_TRUE(uninitialized.observe(rejected,1.0));
  EXPECT_FALSE(uninitialized.streams.at("navigation").boot_known);
  EXPECT_TRUE(uninitialized.retired_boot_ids.empty());
}
TEST(TelemetryTracker, KeepsTrustedNavigationBootWhenForwardIdentityIsMissingOrZero) {
  for (const bool missing : {false,true}) {
    SCOPED_TRACE(missing);
    bx72::TelemetryTracker tracker;
    ASSERT_TRUE(tracker.observe(telemetry("navigation",1,1000000,7),1.0));
    auto unknown=telemetry("navigation",2,1020000,0);
    if (missing) unknown.erase("boot_id");
    ASSERT_TRUE(tracker.observe(unknown,1.02));
    EXPECT_TRUE(tracker.streams.at("navigation").boot_known);
    EXPECT_EQ(tracker.streams.at("navigation").boot_id,7U);
    EXPECT_TRUE(tracker.retired_boot_ids.empty());
    ASSERT_TRUE(tracker.observe(telemetry("navigation",3,1040000,9),1.04));
    ASSERT_EQ(tracker.retired_boot_ids.size(),1U);
    EXPECT_EQ(tracker.retired_boot_ids.front(),7U);
  }
}
TEST(TelemetryTracker, RejectedNavigationCannotBeFirstTrustedRestartVote) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,1500000000U,7),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,1500000000U),10.0));
  auto rejected=telemetry("navigation",1,1000000U,9);
  rejected["rejected"]=true;
  EXPECT_FALSE(tracker.observe(rejected,12.000,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,1005000U),12.005,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,1020000U,9),12.020,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",2,1025000U),12.025,&restarted));
  EXPECT_FALSE(restarted);
  ASSERT_EQ(tracker.epoch,1U);
  EXPECT_TRUE(tracker.observe(telemetry("navigation",3,1040000U,9),12.040,&restarted));
  EXPECT_TRUE(restarted);
  EXPECT_EQ(tracker.epoch,2U);
}
TEST(TelemetryTracker, ReusedRetiredBootCannotBeDistinguishedFromOldPackets) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,4072664482U,7),1.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,4072664482U),1.0));
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1001,4072684482U,9),1.02));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1001,4072684482U),1.02));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",1,380125002U,7),721.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,380145002U,7),721.02,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,380150002U),721.025,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",2,380170002U),721.045,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
  EXPECT_EQ(tracker.streams.at("navigation").boot_id,9U);
}
TEST(TelemetryTracker, RestartDoesNotRetainOrPublishPlansFromOldBoot) {
  const auto planning=[](const char* type,std::uint32_t sequence,std::uint32_t time_us,std::uint32_t boot) {
    auto value=telemetry(type,sequence,time_us,boot); value["boot_id"]=boot; return value;
  };
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,1500000000U,7),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,1500000000U),10.0));
  ASSERT_TRUE(tracker.observe(planning("local_plan",1000,1500000000U,7),10.0));
  ASSERT_TRUE(tracker.observe(planning("local_grid",1000,1500000000U,7),10.0));
  EXPECT_FALSE(tracker.observe(planning("local_plan",1,1000000U,7),12.000,&restarted));
  EXPECT_FALSE(tracker.observe(planning("local_grid",1,1001000U,7),12.001,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",1,1002000U,9),12.002,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,1003000U),12.003,&restarted));
  EXPECT_FALSE(tracker.observe(planning("local_plan",2,1020000U,7),12.020,&restarted));
  EXPECT_FALSE(tracker.observe(planning("local_grid",2,1021000U,7),12.021,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,1022000U,9),12.022,&restarted));
  ASSERT_TRUE(tracker.observe(telemetry("motor",2,1023000U),12.023,&restarted));
  ASSERT_TRUE(restarted);
  EXPECT_EQ(tracker.epoch,2U);
  EXPECT_EQ(tracker.streams.count("local_plan"),0U);
  EXPECT_EQ(tracker.streams.count("local_grid"),0U);
  EXPECT_FALSE(tracker.observe(planning("local_plan",3,1026000U,7),12.026,&restarted));
  EXPECT_FALSE(tracker.observe(planning("local_grid",3,1027000U,7),12.027,&restarted));
  EXPECT_FALSE(tracker.observe(planning("local_plan",3,1028000U,11),12.028,&restarted));
  EXPECT_FALSE(tracker.observe(planning("local_grid",3,1029000U,11),12.029,&restarted));
  auto unknown=planning("local_plan",3,1029000U,9);
  unknown.erase("boot_id");
  EXPECT_FALSE(tracker.observe(unknown,12.029,&restarted));
  EXPECT_TRUE(tracker.observe(planning("local_plan",3,1030000U,9),12.030,&restarted));
  EXPECT_TRUE(tracker.observe(planning("local_grid",3,1031000U,9),12.031,&restarted));
  EXPECT_EQ(tracker.streams.at("local_plan").boot_id,9U);
  EXPECT_EQ(tracker.streams.at("local_grid").boot_id,9U);
}
TEST(TelemetryTracker, InconsistentPlanningVoteDoesNotBlockMatchingIndependentStreams) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,1500000000U,7),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,1500000000U),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("local_plan",1000,1500000000U,7),10.0));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",1,1000000U,9),12.000,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,105000U),12.005,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("local_plan",1,1908000U,9),12.008,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("local_plan",2,1928000U,9),12.028,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,1030000U,9),12.030,&restarted));
  EXPECT_TRUE(tracker.observe(telemetry("motor",2,135000U),12.035,&restarted));
  EXPECT_TRUE(restarted);
  EXPECT_EQ(tracker.epoch,2U);
  EXPECT_EQ(tracker.streams.count("local_plan"),0U);
}
TEST(TelemetryTracker, PlanningVoteCannotBridgeInconsistentNavigationAndMotorTimes) {
  const auto planning=[](const char* type,std::uint32_t sequence,std::uint32_t time_us,std::uint32_t boot) {
    auto value=telemetry(type,sequence,time_us,boot); value["boot_id"]=boot; return value;
  };
  for (const auto* type : {"local_plan","local_grid"}) for (const std::uint32_t boot : {7U,9U}) {
    SCOPED_TRACE(type);
    SCOPED_TRACE(boot);
    bx72::TelemetryTracker tracker;
    bool restarted=false;
    ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,1500000000U,7),10.0));
    ASSERT_TRUE(tracker.observe(telemetry("motor",1000,1500000000U),10.0));
    ASSERT_TRUE(tracker.observe(planning(type,1000,1500000000U,7),10.0));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",1,1000000U,9),12.000,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",1,2500000U),12.005,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",2,1020000U,9),12.020,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",2,2520000U),12.025,&restarted));
    EXPECT_FALSE(tracker.observe(planning(type,1,1750000U,boot),12.030,&restarted));
    EXPECT_FALSE(tracker.observe(planning(type,2,1770000U,boot),12.050,&restarted));
    EXPECT_FALSE(restarted);
    ASSERT_EQ(tracker.epoch,1U);
    // After inconsistent votes expire, two matching streams still recover.
    EXPECT_FALSE(tracker.observe(telemetry("navigation",3,3300000U,9),14.300,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("motor",3,3305000U),14.305,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("navigation",4,3320000U,9),14.320,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("motor",4,3325000U),14.325,&restarted));
    EXPECT_TRUE(restarted);
    EXPECT_EQ(tracker.epoch,2U);
  }
}
TEST(TelemetryTracker, RetiresNavigationBootRegardlessOfGridArrivalOrder) {
  for (const bool grid_first : {false,true}) {
    SCOPED_TRACE(grid_first);
    bx72::TelemetryTracker tracker;
    bool restarted=false;
    ASSERT_TRUE(tracker.observe(telemetry("navigation",1,1000000,7),1.0));
    ASSERT_TRUE(tracker.observe(telemetry("local_grid",1,1000000,7),1.0));
    if (grid_first) {
      // A future grid cannot be published before navigation accepts its boot.
      EXPECT_FALSE(tracker.observe(telemetry("local_grid",2,1020000,9),1.02,&restarted));
      EXPECT_TRUE(tracker.retired_boot_ids.empty());
      EXPECT_TRUE(tracker.observe(telemetry("navigation",2,1020000,9),1.02,&restarted));
      EXPECT_TRUE(tracker.observe(telemetry("local_grid",2,1020000,9),1.02,&restarted));
    } else {
      EXPECT_TRUE(tracker.observe(telemetry("navigation",2,1020000,9),1.02,&restarted));
      EXPECT_TRUE(tracker.observe(telemetry("local_grid",2,1020000,9),1.02,&restarted));
    }
    EXPECT_EQ(tracker.epoch,1U);
    EXPECT_FALSE(restarted);
    ASSERT_EQ(tracker.retired_boot_ids.size(),1U);
    EXPECT_EQ(tracker.retired_boot_ids.front(),7U);
    EXPECT_FALSE(tracker.observe(telemetry("navigation",3,1040000,7),1.04,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("local_grid",3,1040000,7),1.04,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("local_grid",3,1040000,11),1.04,&restarted));
    EXPECT_FALSE(tracker.observe(telemetry("local_plan",3,1040000,11),1.04,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("navigation",3,1040000,9),1.04,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("local_grid",3,1040000,9),1.04,&restarted));
    EXPECT_TRUE(tracker.observe(telemetry("local_plan",3,1040000,9),1.04,&restarted));
  }
}
TEST(TelemetryTracker, SingleLateStreamAndOrdinaryLateBatchesCannotReset) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,200000000,7),1.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,200000000),1.0));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",996,199900000,7),3.5,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",996,199900000),3.5,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",997,199920000,7),3.52,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",997,199920000),3.52,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
  EXPECT_TRUE(tracker.observe(telemetry("navigation",1001,200020000,7),3.6,&restarted));
  EXPECT_FALSE(restarted);

  bx72::TelemetryTracker isolated;
  ASSERT_TRUE(isolated.observe(telemetry("navigation",110399,2208012904U,7),1.0));
  EXPECT_FALSE(isolated.observe(telemetry("navigation",1,1000000,8),3.0,&restarted));
  EXPECT_FALSE(isolated.observe(telemetry("navigation",2,1020000,8),3.02,&restarted));
  EXPECT_FALSE(isolated.observe(telemetry("navigation",3,1040000,8),3.04,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(isolated.epoch,1U);
}
TEST(TelemetryTracker, RequiresConsistentIndependentStreamsAndNewBoot) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,2208012904U,7),1.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,2208012904U),1.0));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",1,1000000,7),2.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,1000000),2.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,1020000,7),2.02,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",2,1020000),2.02,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
  // 新 boot 但两流源时钟不相符，也不能触发重启。
  bx72::TelemetryTracker inconsistent;
  ASSERT_TRUE(inconsistent.observe(telemetry("navigation",1000,2208012904U,7),1.0));
  ASSERT_TRUE(inconsistent.observe(telemetry("motor",1000,2208012904U),1.0));
  EXPECT_FALSE(inconsistent.observe(telemetry("navigation",1,1000000,8),2.0,&restarted));
  EXPECT_FALSE(inconsistent.observe(telemetry("motor",1,5000000),2.0,&restarted));
  EXPECT_FALSE(inconsistent.observe(telemetry("navigation",2,1020000,8),2.02,&restarted));
  EXPECT_FALSE(inconsistent.observe(telemetry("motor",2,5020000),2.02,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(inconsistent.epoch,1U);
  // local_plan 与 navigation 使用同一导航提交，不可当作两条独立来源。
  bx72::TelemetryTracker related;
  ASSERT_TRUE(related.observe(telemetry("navigation",1000,2208012904U,7),1.0));
  ASSERT_TRUE(related.observe(telemetry("local_plan",1000,2208012904U,7),1.0));
  EXPECT_FALSE(related.observe(telemetry("navigation",1,1000000,8),2.0,&restarted));
  EXPECT_FALSE(related.observe(telemetry("local_plan",1,1000000,8),2.0,&restarted));
  EXPECT_FALSE(related.observe(telemetry("navigation",2,1020000,8),2.02,&restarted));
  EXPECT_FALSE(related.observe(telemetry("local_plan",2,1020000,8),2.02,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(related.epoch,1U);
}
TEST(TelemetryTracker, ConfirmsASecondRestartWithoutAcceptingRetiredBoot) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",1000,2208012904U,7),10.0));
  ASSERT_TRUE(tracker.observe(telemetry("motor",1000,2208012904U),10.0));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",1,1000000,8),12.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,1000000),12.0,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,1020000,8),12.02,&restarted));
  ASSERT_TRUE(tracker.observe(telemetry("motor",2,1020000),12.02,&restarted));
  ASSERT_TRUE(restarted);
  EXPECT_TRUE(tracker.observe(telemetry("navigation",500,11020000,8),22.02));
  EXPECT_TRUE(tracker.observe(telemetry("motor",500,11020000),22.02));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",1,100000,9),23.02,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("motor",1,100000),23.02,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",2,120000,9),23.04,&restarted));
  EXPECT_TRUE(tracker.observe(telemetry("motor",2,120000),23.04,&restarted));
  EXPECT_TRUE(restarted);
  EXPECT_EQ(tracker.epoch,3U);
  EXPECT_FALSE(tracker.observe(telemetry("navigation",499,11000000,8),23.06,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.streams.at("navigation").boot_id,9U);
}
TEST(TelemetryTracker, NormalWrapDuplicatesAndShortGapsKeepEpoch) {
  bx72::TelemetryTracker tracker;
  bool restarted=false;
  ASSERT_TRUE(tracker.observe(telemetry("navigation",0xfffffffeU,0xffffff00U,7),1.0));
  EXPECT_TRUE(tracker.observe(telemetry("navigation",0xffffffffU,0xfffffff0U,7),1.0003,&restarted));
  EXPECT_TRUE(tracker.observe(telemetry("navigation",0U,100U,7),1.0005,&restarted));
  EXPECT_FALSE(tracker.observe(telemetry("navigation",0U,100U,7),1.001,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(tracker.epoch,1U);
  bx72::TelemetryTracker short_gap;
  ASSERT_TRUE(short_gap.observe(telemetry("navigation",1000,2208012904U,7),1.0));
  ASSERT_TRUE(short_gap.observe(telemetry("motor",1000,2208012904U),1.0));
  EXPECT_FALSE(short_gap.observe(telemetry("navigation",1,100000,8),1.2,&restarted));
  EXPECT_FALSE(short_gap.observe(telemetry("motor",1,100000),1.2,&restarted));
  EXPECT_TRUE(short_gap.observe(telemetry("navigation",1001,2208312904U,7),1.3,&restarted));
  EXPECT_FALSE(restarted);
  EXPECT_EQ(short_gap.epoch,1U);
}
TEST(NavigationProtocol, SourceTimeStatusAndTelemetrySequenceAreIndependentFromFragmentAck) {
  bx72::Bytes b{'B','X','7','2',1,8,0,84};be(b,5,4);be(b,123,4);
  for(auto v:{1U,2U,3U,4U,0x100100U,0x10007bU})be(b,v,4);
  be(b,1,4);be(b,0x10000,4);
  b.insert(b.end(),{3,0,3,0x87});be(b,100,2);be(b,static_cast<std::uint16_t>(-20),2);
  be(b,0,2);be(b,15,2);be(b,65536,4);be(b,static_cast<std::uint32_t>(-65536),4);
  be(b,24,4);be(b,12000,4);be(b,20000,4);be(b,3,4);
  ASSERT_EQ(b.size(),84U);const auto d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("type"),"navigation");EXPECT_EQ(d.at("protocol"),"nav_full_v1");
  EXPECT_EQ(d.at("now_us"),0x10007bU);EXPECT_EQ(d.at("expiry_us"),0x100100U);
  EXPECT_EQ(d.at("session_token"),0x100010000ULL);
  EXPECT_EQ(d.at("vy_mm_s"),-20);EXPECT_EQ(d.at("goal_y_q16"),-65536);
  EXPECT_EQ(d.at("task_counter_valid"),true);EXPECT_EQ(d.at("last_task_id"),4);
  auto legacy=b;legacy[51]=0;
  EXPECT_THROW(bx72::decode_telemetry(legacy),std::invalid_argument);
  bx72::TelemetryTracker tracker;EXPECT_TRUE(tracker.observe(d,1));EXPECT_FALSE(tracker.observe(d,1.1));
  const auto envelopes=bx72::telemetry_envelopes("local",1,d,1,1.01);
  ASSERT_FALSE(envelopes.empty());EXPECT_EQ(envelopes.front().at("stream"),"navigation");
  b[48]=5;b[49]=0x83;
  const auto rejected=bx72::decode_telemetry(b);
  EXPECT_EQ(rejected.at("rejected"),true);EXPECT_EQ(rejected.at("rejection_reason"),3);
  b[49]=0x44;
  const auto guarded=bx72::decode_telemetry(b);
  EXPECT_EQ(guarded.at("rejected"),false);EXPECT_EQ(guarded.at("guard_error"),4);
  b[50]=4;EXPECT_THROW(bx72::decode_telemetry(b),std::invalid_argument);
  b[50]=2;b[48]=10;EXPECT_THROW(bx72::decode_telemetry(b),std::invalid_argument);
}
TEST(NavigationProtocol, ZeroDeadlineRemainsKnownForActiveTasksAcrossClockWrap) {
  bx72::Bytes b{'B','X','7','2',1,8,0,84};be(b,5,4);be(b,0xfffffff0U,4);
  for(auto v:{1U,2U,3U,3U,0U,0xfffffff0U})be(b,v,4);
  be(b,1,8);b.insert(b.end(),{3,0,2,0xc6});b.resize(84,0);
  for (const auto mode : {2U,3U}) for (const auto status : {1U,2U,3U,5U,8U,9U}) {
    b[48]=status;b[50]=mode;
    const auto decoded=bx72::decode_telemetry(b);
    EXPECT_TRUE(decoded.at("deadline_known").get<bool>());
    EXPECT_EQ(decoded.at("expiry_us"),0);
  }
  for (const auto status : {0U,4U,6U,7U}) {
    b[48]=status;
    EXPECT_FALSE(bx72::decode_telemetry(b).at("deadline_known").get<bool>());
  }
  b[48]=3;b[50]=1;
  EXPECT_FALSE(bx72::decode_telemetry(b).at("deadline_known").get<bool>());
  b[48]=8;
  EXPECT_FALSE(bx72::decode_telemetry(b).at("deadline_known").get<bool>());
  b[35]=1;
  EXPECT_TRUE(bx72::decode_telemetry(b).at("deadline_known").get<bool>());
}
TEST(NavigationProtocol, LocalPlanSegmentDecodesAndForwards) {
  bx72::Bytes b{'B','X','7','2',1,9,0,56};
  for (auto v : {7U, 1000U, 42U, 9U, 17U, 950U,
                 static_cast<std::uint32_t>(-1250), 500U, 3000U,
                 static_cast<std::uint32_t>(-250), 5U, 99U}) be(b,v,4);
  ASSERT_EQ(b.size(),56U);
  const auto d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("type"),"local_plan");
  EXPECT_EQ(d.at("protocol"),"nav_local_segment_v1");
  EXPECT_EQ(d.at("boot_id"),42); EXPECT_EQ(d.at("odom_domain"),9);
  EXPECT_EQ(d.at("task_id"),17); EXPECT_EQ(d.at("plan_stamp_us"),950);
  EXPECT_EQ(d.at("time_us"),1000); EXPECT_EQ(d.at("polar_generation"),99);
  EXPECT_EQ(d.at("start_x_mm"),-1250); EXPECT_EQ(d.at("end_y_mm"),-250);
  EXPECT_DOUBLE_EQ(d.at("points").at(0).at("x").get<double>(),-1.25);
  EXPECT_DOUBLE_EQ(d.at("points").at(1).at("y").get<double>(),-.25);
  EXPECT_EQ(d.at("frame_id"),"odom"); EXPECT_EQ(d.at("source"),"fpga");
  EXPECT_TRUE(d.at("valid").get<bool>());
  EXPECT_TRUE(d.at("guard_stop").get<bool>());
  bx72::TelemetryTracker tracker;
  EXPECT_TRUE(tracker.observe(d,1)); EXPECT_FALSE(tracker.observe(d,1.1));
  const auto envelopes=bx72::telemetry_envelopes("local",1,d,1,1.01);
  ASSERT_EQ(envelopes.size(),1U); EXPECT_EQ(envelopes[0].at("data"),d);
  b[51]=13;
  const auto grid=bx72::decode_telemetry(b);
  EXPECT_TRUE(grid.at("grid_direction_preview").get<bool>());
  EXPECT_EQ(grid.at("map_version"),99);
  EXPECT_FALSE(grid.contains("polar_generation"));
  b[51]=16; EXPECT_THROW(bx72::decode_telemetry(b),std::invalid_argument);
  b[51]=0; b.pop_back(); EXPECT_THROW(bx72::decode_telemetry(b),std::invalid_argument);
}
TEST(NavigationProtocol, PolarV2KeepsGenerationAndCandidateNamesSeparate) {
  bx72::Bytes b{'B','X','7','2',1,8,0,84};be(b,5,4);be(b,123,4);
  for(auto v:{1U,2U,3U,4U,0x100100U,0x10007bU})be(b,v,4);
  be(b,1,4);be(b,0x10000,4);
  b.insert(b.end(),{3,0,3,0xc7});be(b,100,2);be(b,static_cast<std::uint16_t>(-20),2);
  be(b,0,2);be(b,72,2);be(b,65536,4);be(b,static_cast<std::uint32_t>(-65536),4);
  be(b,24,4);be(b,12000,4);be(b,20000,4);be(b,0xfffffff1U,4);
  ASSERT_EQ(b.size(),84U);const auto d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("protocol"),"nav_polar_v2");
  EXPECT_EQ(d.at("polar_candidates_evaluated"),72);
  EXPECT_EQ(d.at("polar_age_us"),12000);
  EXPECT_EQ(d.at("polar_generation"),0xfffffff1U);
  EXPECT_FALSE(d.contains("planner_expansions"));
  EXPECT_FALSE(d.contains("map_age_us"));
  EXPECT_FALSE(d.contains("map_version"));
  const auto envelopes=bx72::telemetry_envelopes("local",1,d,1,1.01);
  ASSERT_FALSE(envelopes.empty());
  bool generation_preserved=false;
  for(const auto& envelope:envelopes)
    if(envelope.at("data").contains("polar_generation"))
      generation_preserved=envelope.at("data").at("polar_generation")==0xfffffff1U;
  EXPECT_TRUE(generation_preserved);
  auto reserved=b;reserved[51]|=0x08;
  EXPECT_THROW(bx72::decode_telemetry(reserved),std::invalid_argument);
}
TEST(NavigationProtocol, GridV1UsesCandidateAndMapNamesAndRejectsMixedVersions) {
  bx72::Bytes b{'B','X','7','2',1,8,0,84};be(b,5,4);be(b,123,4);
  for(auto v:{1U,2U,3U,4U,0x100100U,0x10007bU})be(b,v,4);
  be(b,1,4);be(b,0x10000,4);
  b.insert(b.end(),{3,0,3,0xa7});be(b,100,2);be(b,static_cast<std::uint16_t>(-20),2);
  be(b,0,2);be(b,7,2);be(b,65536,4);be(b,static_cast<std::uint32_t>(-65536),4);
  be(b,24,4);be(b,12000,4);be(b,20000,4);be(b,0xfffffff1U,4);
  ASSERT_EQ(b.size(),84U);
  const auto decoded=bx72::decode_telemetry(b);
  EXPECT_EQ(decoded.at("protocol"),"nav_grid_v1");
  EXPECT_EQ(decoded.at("candidates_evaluated"),7);
  EXPECT_EQ(decoded.at("map_age_us"),12000);
  EXPECT_EQ(decoded.at("map_version"),0xfffffff1U);
  EXPECT_FALSE(decoded.contains("planner_expansions"));
  EXPECT_FALSE(decoded.contains("polar_generation"));
  auto mixed=b;mixed[51]|=0x40;
  EXPECT_THROW(bx72::decode_telemetry(mixed),std::invalid_argument);
  auto reserved=b;reserved[51]|=0x08;
  EXPECT_THROW(bx72::decode_telemetry(reserved),std::invalid_argument);
}
TEST(NavigationProtocol, VelocityOnlyReportsGuardAndRejectsMixedArchitecture) {
  bx72::Bytes packet{'B','X','7','2',1,8,0,84};
  packet.resize(84,0);
  packet[19]=7; packet[23]=1; packet[39]=1;
  packet[50]=1; packet[51]=0x97;
  const auto guarded=bx72::decode_telemetry(packet);
  EXPECT_EQ(guarded.at("protocol"),"velocity_only_v1");
  EXPECT_TRUE(guarded.at("guard_stop").get<bool>());
  EXPECT_TRUE(guarded.at("pose_valid").get<bool>());
  packet[51]=0x96;
  EXPECT_FALSE(bx72::decode_telemetry(packet).at("guard_stop").get<bool>());
  packet[51]=0xb6;
  EXPECT_THROW(bx72::decode_telemetry(packet),std::invalid_argument);
}
TEST(NavigationStatus, DistinguishesCancelledFromWaitingAuto) {
  EXPECT_TRUE(bx72::navigation_status_active(9));
  EXPECT_FALSE(bx72::navigation_status_terminal(9));
  EXPECT_FALSE(bx72::navigation_status_active(7));
  EXPECT_TRUE(bx72::navigation_status_terminal(7));
  EXPECT_TRUE(bx72::navigation_status_active(5));
  EXPECT_FALSE(bx72::navigation_status_terminal(5));
}
TEST(FusionTelemetry, RequiresFullLayoutMarkerAndReportsFiveRealCovariances) {
  bx72::Bytes b{'B','X','7','2',1,6,0,120};be(b,7,4);be(b,0xfffffff0U,4);
  for (const auto value : {1000U, static_cast<std::uint32_t>(-2000), 500U,
                           200U, static_cast<std::uint32_t>(-100U), 300U,
                           10U, 200U, static_cast<std::uint32_t>(-300U)}) be(b,value,4);
  for (const auto value : {0x3f800000U,0x40000000U,0x40400000U,
                           0x40800000U,0x40a00000U}) be(b,value,4);
  be(b,0xd0000001U,4); // 单位、完整布局、自检通过标记和pose有效
  for (unsigned i=0;i<11;++i) be(b,i,4);
  ASSERT_EQ(b.size(),120U);
  const auto diagnostics=(1U<<30)|(1U<<29)|(17U<<23)|(123U<<8)|5U;
  for(unsigned i=0;i<4;++i)b[112+i]=static_cast<std::uint8_t>(diagnostics>>((3-i)*8));
  const auto decoded=bx72::decode_telemetry(b);
  EXPECT_DOUBLE_EQ(decoded.at("state").at("x_m"),1.0);
  EXPECT_DOUBLE_EQ(decoded.at("state").at("y_m"),-2.0);
  EXPECT_DOUBLE_EQ(decoded.at("state").at("roll_rad"),0.2);
  EXPECT_EQ(decoded.at("covariance_diag").size(),5U);
  EXPECT_DOUBLE_EQ(decoded.at("covariance_diag").at(0),1e-6);
  EXPECT_DOUBLE_EQ(decoded.at("covariance_diag").at(2),3.0);
  EXPECT_EQ(decoded.at("map_ray_overflow"),true);
  EXPECT_EQ(decoded.at("map_shift_pending"),true);
  EXPECT_EQ(decoded.at("map_ray_queue_depth"),17);
  EXPECT_EQ(decoded.at("map_ray_drops"),123);
  EXPECT_EQ(decoded.at("map_update_mode"),"ray_queue");
  EXPECT_TRUE(decoded.at("map_raster_row").is_null());
  EXPECT_EQ(decoded.at("csm_rejected"),10);
  // 相同位域在新标记下明确表示扫描进度/输入缺口，不能混解成空队列。
  auto raster=b;raster[74]|=0x80;
  const auto raster_decoded=bx72::decode_telemetry(raster);
  EXPECT_EQ(raster_decoded.at("map_update_mode"),"visibility_raster");
  EXPECT_EQ(raster_decoded.at("map_raster_row"),17);
  EXPECT_EQ(raster_decoded.at("map_input_gaps"),123);
  EXPECT_EQ(raster_decoded.at("map_input_gap_latched"),true);
  EXPECT_TRUE(raster_decoded.at("map_ray_queue_depth").is_null());
  EXPECT_TRUE(raster_decoded.at("map_ray_drops").is_null());
  EXPECT_TRUE(raster_decoded.at("map_ray_overflow").is_null());
  auto legacy=b;legacy[72]=0x80;legacy[73]=0x00;
  EXPECT_THROW(bx72::decode_telemetry(legacy),std::invalid_argument);
}

TEST(FusionTelemetry, PolarV2DecodesEstimatesScanMatchAndGapStatusWithoutLegacyAliases) {
  bx72::Bytes b{'B','X','7','2',1,6,0,120};be(b,8,4);be(b,0xfffffff0U,4);
  for (const auto value : {1000U, static_cast<std::uint32_t>(-2000), 500U,
                           200U, static_cast<std::uint32_t>(-100U), 300U,
                           10U, 200U, static_cast<std::uint32_t>(-300U)}) be(b,value,4);
  for (const auto value : {0x41280000U,0x41a20000U,0x40400000U}) be(b,value,4); // 10.5 mm, 20.25 mm, 3 mrad
  be(b,0xfffffff1U,4);be(b,7,4);
  be(b,0xc0004001U,4); // Layout marker, architecture marker, polar-v2 and valid pose.
  be(b,101,4);be(b,0,4);be(b,1000,4);be(b,0xfffffff0U,4);
  be(b,7,4);be(b,8,4);be(b,0xf0000001U,4);be(b,9,4);be(b,10,4);
  const auto nav_status=3U|(1U<<8)|(1U<<9)|(1U<<10)|(1U<<11)|(1U<<12)|(1U<<13)|(1U<<14);
  be(b,nav_status,4);be(b,0xe0000002U,4);
  ASSERT_EQ(b.size(),120U);
  const auto decoded=bx72::decode_telemetry(b);
  EXPECT_EQ(decoded.at("architecture"),"polar_v2");
  EXPECT_EQ(decoded.at("polar_navigation_architecture"),true);
  EXPECT_EQ(decoded.at("map_update_mode"),"polar_gap");
  ASSERT_EQ(decoded.at("error_estimate").size(),3U);
  EXPECT_DOUBLE_EQ(decoded.at("error_estimate").at(0),10.5);
  EXPECT_DOUBLE_EQ(decoded.at("error_estimate").at(1),20.25);
  EXPECT_DOUBLE_EQ(decoded.at("error_estimate").at(2),3.0);
  EXPECT_EQ(decoded.at("ros_odometry_covariance_model"),"polar_error_estimate_squared_heuristic");
  EXPECT_EQ(decoded.at("lidar_raw_samples").get<std::uint32_t>(),0xfffffff1U);
  EXPECT_EQ(decoded.at("lidar_reduced_samples").get<std::uint32_t>(),7U);
  EXPECT_EQ(decoded.at("scan_match_accepted").get<std::uint32_t>(),0xf0000001U);
  EXPECT_EQ(decoded.at("scan_match_rejected").get<std::uint32_t>(),0xe0000002U);
  EXPECT_EQ(decoded.at("polar_scan_fresh"),true);
  EXPECT_EQ(decoded.at("polar_complete_coverage"),true);
  EXPECT_EQ(decoded.at("polar_gap_latched"),true);
  EXPECT_EQ(decoded.at("boundary_following"),true);
  EXPECT_EQ(decoded.at("progress_timeout_or_loop"),true);
  EXPECT_EQ(decoded.at("unknown_in_swept_corridor"),true);
  EXPECT_EQ(decoded.at("polar_recovery_required"),true);
  EXPECT_FALSE(decoded.contains("covariance_diag"));
  EXPECT_FALSE(decoded.contains("csm_accepted"));
  EXPECT_FALSE(decoded.contains("csm_rejected"));
  EXPECT_FALSE(decoded.contains("map_ray_queue_depth"));
  auto stopped=b;stopped[112]|=0x80;
  EXPECT_EQ(bx72::decode_telemetry(stopped).at("software_motion_stop"),true);
  auto reserved=b;reserved[112]|=0x40;
  EXPECT_THROW(bx72::decode_telemetry(reserved),std::invalid_argument);
}

TEST(FusionTelemetry, GridV1RetainsEstimatesAndUsesGridDiagnostics) {
  bx72::Bytes b{'B','X','7','2',1,6,0,120};be(b,8,4);be(b,100000,4);
  for (const auto value : {1000U,static_cast<std::uint32_t>(-2000),500U,200U,
                           static_cast<std::uint32_t>(-100),300U,10U,200U,
                           static_cast<std::uint32_t>(-300)}) be(b,value,4);
  for (const auto value : {0x41280000U,0x41a20000U,0x40400000U,0xfffffff1U,7U}) be(b,value,4);
  be(b,0xc000c001U,4);
  for(const auto value : {101U,0U,1000U,99999U,7U,8U,0xf0000001U,9U,10U,
                          0x8000ff03U,0xe0000002U}) be(b,value,4);
  ASSERT_EQ(b.size(),120U);
  const auto decoded=bx72::decode_telemetry(b);
  EXPECT_EQ(decoded.at("architecture"),"local_grid_v1");
  EXPECT_EQ(decoded.at("map_update_mode"),"local_grid");
  EXPECT_EQ(decoded.at("polar_navigation_architecture"),false);
  EXPECT_DOUBLE_EQ(decoded.at("error_estimate").at(0),10.5);
  EXPECT_EQ(decoded.at("lidar_raw_samples").get<std::uint32_t>(),0xfffffff1U);
  EXPECT_EQ(decoded.at("lidar_reduced_samples"),7);
  EXPECT_EQ(decoded.at("scan_match_accepted").get<std::uint32_t>(),0xf0000001U);
  for (const auto* name : {"map_fresh","map_ready","map_gap_latched","candidate_planning",
                           "progress_blocked","unknown_path","map_recovery_required",
                           "grid_v1_diagnostic","software_motion_stop"})
    EXPECT_EQ(decoded.at(name),true) << name;
  EXPECT_FALSE(decoded.contains("covariance_diag"));
  EXPECT_FALSE(decoded.contains("map_raster_row"));
  EXPECT_FALSE(decoded.contains("polar_scan_fresh"));
  EXPECT_TRUE(decoded.at("map_lidar_drops").is_null());
  EXPECT_EQ(decoded.at("lidar_drops_scope"),"csm_reduced");
  auto runtime=b;
  const auto diagnostics=0x00310000U;
  for (unsigned i=0;i<4;++i) runtime[112+i]|=(diagnostics>>(24-8*i))&0xffU;
  const auto runtime_decoded=bx72::decode_telemetry(runtime);
  EXPECT_EQ(runtime_decoded.at("map_lidar_drops"),7);
  EXPECT_EQ(runtime_decoded.at("lidar_drops_scope"),"map_raw");
  runtime[113]|=0x40; // nav_status bit22：建图与CSM共用约简队列。
  const auto shared_decoded=bx72::decode_telemetry(runtime);
  EXPECT_EQ(shared_decoded.at("map_input_mode"),"shared_nearest_5deg");
  EXPECT_EQ(shared_decoded.at("lidar_drops_scope"),"shared_reduced");
  EXPECT_EQ(runtime_decoded.at("map_gap_reason_name"),"fifo_overflow");
  EXPECT_EQ(runtime_decoded.at("start_confirmation_lost"),true);
  auto reserved=b;reserved[112]|=0x40;
  EXPECT_THROW(bx72::decode_telemetry(reserved),std::invalid_argument);
}

TEST(FusionTelemetry, VelocityOnlyKeepsPoseEstimatesWithoutMapDiagnostics) {
  bx72::Bytes packet{'B','X','7','2',1,6,0,120};
  packet.resize(120,0);
  packet[72]=0xc0; packet[74]=0x60; packet[75]=0x01;
  const auto decoded=bx72::decode_telemetry(packet);
  EXPECT_EQ(decoded.at("architecture"),"velocity_only_v1");
  EXPECT_TRUE(decoded.at("pose_valid").get<bool>());
  EXPECT_EQ(decoded.at("error_estimate").size(),3U);
  EXPECT_FALSE(decoded.contains("map_update_mode"));
}

TEST(TelemetryEnvelope, ExactDatagramLimitAndFragmentedArrayPreserveData) {
  const bx72::Json small={{"type","health"},{"sequence",0xfffffff1U},
      {"time_us",0xffffff00U},{"cpu_ready",true},
      {"camera_calibration",{{"state",12},{"ready",true},{"fault",false}}}};
  const auto ordinary=bx72::telemetry_envelopes("live",1,small,1,1.01);
  ASSERT_EQ(ordinary.size(),1U);
  const auto exact=bx72::telemetry_envelopes("live",1,small,1,1.01,ordinary[0].dump().size());
  ASSERT_EQ(exact.size(),1U);EXPECT_EQ(exact[0].at("data"),small);
  EXPECT_FALSE(exact[0].contains("parts"));
  auto large=small;large["values"]=bx72::Json::array();
  for(int i=0;i<256;++i)large["values"].push_back(i);
  const auto split=bx72::telemetry_envelopes("live",1,large,1,1.01,512);
  ASSERT_GT(split.size(),1U);bx72::Json restored=bx72::Json::array();
  for(std::size_t i=0;i<split.size();++i) {
    EXPECT_LE(split[i].dump().size(),512U);
    EXPECT_EQ(split[i].at("part"),i);EXPECT_EQ(split[i].at("parts"),split.size());
    EXPECT_EQ(split[i].at("data").at("sequence"),small.at("sequence"));
    if(split[i].at("data").contains("values"))
      for(const auto& value:split[i].at("data").at("values"))restored.push_back(value);
  }
  EXPECT_EQ(restored,large.at("values"));
}

TEST(TelemetryEnvelope, RejectsOversizedAsciiAndUtf8Strings) {
  bx72::Json data={{"type","navigation_task"},{"sequence",1},{"time_us",2},
                   {"state",{{"generation",std::string(3000,'a')}}}};
  EXPECT_THROW(bx72::telemetry_envelopes("live",1,data,1,1.01),std::length_error);

  std::string chinese;
  for (int i=0;i<1000;++i) chinese+=u8"中";
  data["state"]["generation"]=chinese;
  EXPECT_THROW(bx72::telemetry_envelopes("live",1,data,1,1.01),std::length_error);
}

TEST(TelemetryEnvelope, PreservesUtf8StringAlongsideFragmentedArray) {
  const std::string generation=u8"地图一代";
  bx72::Json data={{"type","navigation_task"},{"sequence",1},{"time_us",2},
                   {"state",{{"generation",generation},{"route",bx72::Json::array()}}}};
  for (int i=0;i<256;++i) data["state"]["route"].push_back(i);
  const auto envelopes=bx72::telemetry_envelopes("live",1,data,1,1.01,512);
  ASSERT_GT(envelopes.size(),1U);
  unsigned generation_count=0, route_count=0;
  for (const auto& envelope:envelopes) {
    EXPECT_LE(envelope.dump().size(),512U);
    const auto& part=envelope.at("data");
    if (!part.contains("state")) continue;
    const auto& state=part.at("state");
    if (state.contains("generation")) {
      ++generation_count;
      EXPECT_EQ(state.at("generation"),generation);
    }
    if (state.contains("route")) route_count+=state.at("route").size();
  }
  EXPECT_EQ(generation_count,1U);
  EXPECT_EQ(route_count,256U);
}

TEST(HealthTelemetry, CpuSignatureSeparatesStatusFromLegacyQueueCounter) {
  bx72::Bytes b{'B','X','7','2',1,7,0,128};be(b,8,4);be(b,100000,4);
  const std::uint32_t guard_gray=(12U<<16)|11U; // binary bad=8, good=13, each Gray-decoded separately.
  for(unsigned i=0;i<27;++i)be(b,i==14 ? 0xc3245a03U : (i==15 ? guard_gray : 0U),4);
  be(b,0x4752005bU,4); // ready、fault、busy、fault_code=5，保留位为零。
  auto d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("cpu_health_valid"),true);EXPECT_EQ(d.at("cpu_ready"),true);
  EXPECT_EQ(d.at("cpu_fault"),true);EXPECT_EQ(d.at("cpu_busy"),true);
  EXPECT_EQ(d.at("cpu_fault_code"),5);EXPECT_TRUE(d.at("queue_dropped").is_null());
  EXPECT_TRUE(d.at("ddr_gray_7").is_null());EXPECT_TRUE(d.at("ddr_gray_8").is_null());
  EXPECT_EQ(d.at("camera_autocal_diag").at("reason_name"),"bad_raw_eof");
  EXPECT_EQ(d.at("camera_autocal_diag").at("tap0"),0x24);
  EXPECT_EQ(d.at("camera_autocal_diag").at("tap1"),0x5a);
  EXPECT_EQ(d.at("camera_autocal_diag").at("retry_count"),3);
  EXPECT_EQ(d.at("camera_guard_frame_counts").at("bad_frames"),8);
  EXPECT_EQ(d.at("camera_guard_frame_counts").at("good_frames"),13);
  EXPECT_EQ(d.at("camera_guard_failure_diagnostics_valid"),false);
  EXPECT_FALSE(d.contains("camera_guard_failure_sticky"));
  // 新解码器先兼容旧格式；能力位为1时逐位检查所有16种组合。
  for(unsigned flags=0;flags<16;++flags) {
    b[126]=static_cast<std::uint8_t>((flags<<4)|(b[126]&15U));
    b[127]|=4;
    const auto x=bx72::decode_telemetry(b);
    ASSERT_EQ(x.at("camera_guard_failure_diagnostics_valid"),true);
    const auto& cause=x.at("camera_guard_failure_sticky");
    EXPECT_EQ(cause.at("source_incomplete"),bool(flags&1));
    EXPECT_EQ(cause.at("source_complete_pixel_geometry"),bool(flags&2));
    EXPECT_EQ(cause.at("end_timeout"),bool(flags&4));
    EXPECT_EQ(cause.at("no_fe_timeout"),bool(flags&8));
    EXPECT_EQ(x.at("cpu_fault_code"),5);
    EXPECT_EQ(x.at("camera_guard_frame_counts").at("bad_frames"),8);
    EXPECT_EQ(x.at("camera_guard_frame_counts").at("good_frames"),13);
  }
  // 没有能力位时，旧格式4个保留位的任何非零组合仍必须拒收。
  for(unsigned flags=1;flags<16;++flags) {
    b[126]=static_cast<std::uint8_t>((flags<<4)|(b[126]&15U));
    b[127]&=~4U;
    EXPECT_THROW(bx72::decode_telemetry(b),std::invalid_argument);
  }
  b[124]=0;b[125]=0;b[126]=0;b[127]=7;
  d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("cpu_health_valid"),false);EXPECT_EQ(d.at("queue_dropped"),7);
  EXPECT_FALSE(d.contains("cpu_ready"));
}

TEST(HealthTelemetry, BothFormatsRequireExact128ByteDatagrams) {
  for (const std::uint32_t capability : {0U, 4U}) {
    bx72::Bytes b{'B','X','7','2',1,7,0,128};be(b,8,4);be(b,100000,4);
    for(unsigned i=0;i<27;++i)be(b,0,4);
    be(b,0x47520001U|capability,4);
    ASSERT_EQ(b.size(),128U);
    EXPECT_NO_THROW(bx72::decode_telemetry(b));
    auto short_packet=b;short_packet.pop_back();short_packet[7]=127;
    EXPECT_THROW(bx72::decode_telemetry(short_packet),std::invalid_argument);
    auto long_packet=b;long_packet.push_back(0);long_packet[7]=129;
    EXPECT_THROW(bx72::decode_telemetry(long_packet),std::invalid_argument);
    auto wrong_declared=b;wrong_declared[7]=127;
    EXPECT_THROW(bx72::decode_telemetry(wrong_declared),std::invalid_argument);
  }
}

TEST(HealthTelemetry, CameraInitializationDoesNotImplyCalibrationReady) {
  bx72::Bytes b{'B','X','7','2',1,7,0,128};be(b,85,4);be(b,85999999,4);
  for(unsigned i=0;i<28;++i)be(b,0,4);
  const auto word=[&](unsigned index,std::uint32_t value) {
    for(unsigned i=0;i<4;++i)b[16+index*4+i]=static_cast<std::uint8_t>(value>>((3-i)*8));
  };
  word(0,0x003ba31fU);word(4,0x42e80000U);word(27,0x47520001U);
  auto d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("video_ready").at("camera_initialized"),true);
  const auto &c=d.at("camera_calibration");
  EXPECT_EQ(c.at("state"),4);EXPECT_EQ(c.at("ready"),false);EXPECT_EQ(c.at("fault"),false);
  EXPECT_EQ(c.at("scan_row"),23);EXPECT_EQ(c.at("scan_col"),8);
  EXPECT_EQ(d.at("video_rejected_mod256").at("camera_bad"),163);
  EXPECT_EQ(d.at("video_rejected_mod256").at("pixel_count"),59);
  EXPECT_EQ(d.at("video_rejected_mod256").at("fifo_full"),0);
  word(4,0xc8000040U);
  d=bx72::decode_telemetry(b);
  EXPECT_EQ(d.at("camera_calibration").at("state"),12);
  EXPECT_EQ(d.at("camera_calibration").at("ready"),true);
  EXPECT_EQ(d.at("camera_calibration").at("verified_bursts"),8);
  EXPECT_FALSE(d.at("camera_calibration").contains("verified_frames"));
  word(27,7);
  EXPECT_FALSE(bx72::decode_telemetry(b).contains("camera_calibration"));
}
