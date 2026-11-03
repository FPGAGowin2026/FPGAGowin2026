#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "ugv_bridge/video.hpp"

namespace {

constexpr std::size_t kFrameBytes = bx72::video_width * bx72::video_height * 2U;

bx72::Bytes packed_frame(
  const std::string& format, std::uint8_t y, std::uint8_t u, std::uint8_t v)
{
  bx72::Bytes raw(kFrameBytes);
  for (std::size_t i = 0; i < raw.size(); i += 4) {
    if (format == "yuyv") {
      raw[i] = y;
      raw[i + 1] = u;
      raw[i + 2] = y;
      raw[i + 3] = v;
    } else {
      raw[i] = u;
      raw[i + 1] = y;
      raw[i + 2] = v;
      raw[i + 3] = y;
    }
  }
  return raw;
}

bx72::Bytes rtp_packet(
  std::uint8_t payload_type, std::uint16_t sequence, std::uint32_t timestamp,
  bool marker, const bx72::Bytes& payload)
{
  bx72::Bytes packet(12, 0);
  packet[0] = 0x80;
  packet[1] = payload_type | (marker ? 0x80 : 0);
  packet[2] = static_cast<std::uint8_t>(sequence >> 8);
  packet[3] = static_cast<std::uint8_t>(sequence);
  packet[4] = static_cast<std::uint8_t>(timestamp >> 24);
  packet[5] = static_cast<std::uint8_t>(timestamp >> 16);
  packet[6] = static_cast<std::uint8_t>(timestamp >> 8);
  packet[7] = static_cast<std::uint8_t>(timestamp);
  std::copy_n("ACG6", 4, packet.begin() + 8);
  packet.insert(packet.end(), payload.begin(), payload.end());
  return packet;
}

TEST(Yuv422Conversion, ConvertsYuyvRedToBgr)
{
  const auto bgr = bx72::yuv422_to_bgr(packed_frame("yuyv", 62, 101, 240), "yuyv");
  ASSERT_EQ(bgr.cols, bx72::video_width);
  ASSERT_EQ(bgr.rows, bx72::video_height);
  ASSERT_EQ(bgr.type(), CV_8UC3);
  const auto pixel = bgr.at<cv::Vec3b>(0, 0);
  EXPECT_LT(pixel[0], 5);
  EXPECT_LT(pixel[1], 5);
  EXPECT_GT(pixel[2], 250);
}

TEST(Yuv422Conversion, ConvertsUyvyRedToBgr)
{
  const auto bgr = bx72::yuv422_to_bgr(packed_frame("uyvy", 62, 101, 240), "uyvy");
  const auto pixel = bgr.at<cv::Vec3b>(0, 0);
  EXPECT_LT(pixel[0], 5);
  EXPECT_LT(pixel[1], 5);
  EXPECT_GT(pixel[2], 250);
}

TEST(Yuv422Conversion, ConvertsUyvyGrayToEqualBgrChannels)
{
  const auto bgr = bx72::yuv422_to_bgr(packed_frame("uyvy", 128, 128, 128), "uyvy");
  const auto pixel = bgr.at<cv::Vec3b>(100, 100);
  EXPECT_LE(std::abs(static_cast<int>(pixel[0]) - static_cast<int>(pixel[1])), 1);
  EXPECT_LE(std::abs(static_cast<int>(pixel[1]) - static_cast<int>(pixel[2])), 1);
}

TEST(Yuv422Conversion, PreservesBothLumaSamplesInEachPair)
{
  for (const auto& format : {std::string("yuyv"), std::string("uyvy")}) {
    auto raw = packed_frame(format, 16, 128, 128);
    raw[format == "yuyv" ? 2 : 3] = 235;
    const auto bgr = bx72::yuv422_to_bgr(raw, format);
    for (int channel = 0; channel < 3; ++channel) {
      EXPECT_EQ(bgr.at<cv::Vec3b>(0, 0)[channel], 0);
      EXPECT_EQ(bgr.at<cv::Vec3b>(0, 1)[channel], 255);
      EXPECT_EQ(bgr.at<cv::Vec3b>(0, 2)[channel], 0);
    }
  }
}

TEST(Yuv422Conversion, RejectsWrongByteCountAndFormat)
{
  EXPECT_THROW(bx72::yuv422_to_bgr(bx72::Bytes(kFrameBytes - 1), "yuyv"),
    std::invalid_argument);
  EXPECT_THROW(bx72::yuv422_to_bgr(bx72::Bytes(kFrameBytes + 1), "uyvy"),
    std::invalid_argument);
  EXPECT_THROW(bx72::yuv422_to_bgr({}, "rgb565"), std::invalid_argument);
}

TEST(Yuv422Rtp, RequiresPayloadType97)
{
  bx72::RtpFrames frames(2, 1);
  EXPECT_FALSE(frames.feed(rtp_packet(96, 0, 1, true, {10, 128, 10, 128})).has_value());
  EXPECT_EQ(frames.packets, 0U);
  const auto complete = frames.feed(
    rtp_packet(bx72::video_payload_type, 0, 2, true, {16, 128, 16, 128}));
  ASSERT_TRUE(complete.has_value());
  EXPECT_EQ(complete->size(), 4U);
}

TEST(PcMonitoring, MonitorDoesNotArmAndManualControlRequiresANewZero)
{
  bx72::PcLease lease;
  bx72::Json message{{"v",1},{"type","control"},{"session","web"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  ASSERT_TRUE(lease.accept(message,"127.0.0.1:1234",1.0));
  EXPECT_FALSE(lease.armed);
  message["seq"]=2; message["vx"]=10;
  EXPECT_FALSE(lease.accept(message,"127.0.0.1:1234",1.1));
  message["take_control"]=true;
  EXPECT_FALSE(lease.accept(message,"127.0.0.1:1234",1.1));
  message["vx"]=0;
  ASSERT_TRUE(lease.accept(message,"127.0.0.1:1234",1.1));
  EXPECT_TRUE(lease.armed);
  lease.expire(1.31); EXPECT_FALSE(lease.armed);
  message["seq"]=3; ASSERT_TRUE(lease.accept(message,"127.0.0.1:1234",1.32));
  EXPECT_FALSE(lease.accept(message,"127.0.0.1:1234",1.4));
  message["seq"]=4; message["vx"]=10;
  EXPECT_TRUE(lease.accept(message,"127.0.0.1:1234",1.4));
  message["seq"]=5; message["vx"]=0; message["take_control"]=false;
  EXPECT_TRUE(lease.accept(message,"127.0.0.1:1234",1.5));
  EXPECT_FALSE(lease.armed);
}

TEST(PcMonitoring, MultipleIdleViewersCannotChangeTheActiveControlLease)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto owner=bx72::endpoint("127.0.0.1",31001);
  const auto viewer=bx72::endpoint("127.0.0.1",31002);
  bx72::Json message{{"v",1},{"type","control"},{"session","owner"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,owner,1.0),bx72::PcInputResult::Observer);
  message["seq"]=2; message["take_control"]=true; message["claim"]=true;
  message["claim_nonce"]=observers.find(owner)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,owner,1.005),bx72::PcInputResult::Owner);
  message["seq"]=3; message["vx"]=80; message.erase("claim");
  frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,owner,1.01),bx72::PcInputResult::Owner);
  message["session"]="viewer"; message["seq"]=1; message["vx"]=0; message["take_control"]=false;
  frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,1.02),bx72::PcInputResult::Observer);
  EXPECT_EQ(observers.entries().size(),2U);
  EXPECT_EQ(lease.session,"owner");
  EXPECT_EQ(lease.peer,bx72::peer_key(owner));
  EXPECT_EQ(lease.seq,3U);
  EXPECT_EQ(lease.velocity[0],80);
  EXPECT_TRUE(lease.armed);
  EXPECT_DOUBLE_EQ(lease.last,1.01);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,1.03),bx72::PcInputResult::Rejected);
  message["vx"]=1;
  EXPECT_FALSE(bx72::parse_pc_control(message));
  message["vx"]=0; message["seq"]=2;
  frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,1.04,true,true),bx72::PcInputResult::Observer);
  EXPECT_EQ(lease.session,"owner");
  EXPECT_TRUE(lease.armed);
  EXPECT_EQ(lease.velocity[0],80);
}

TEST(PcMonitoring, ViewerSessionRestartAndOwnerNavigationHeartbeatRemainIndependent)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto owner=bx72::endpoint("127.0.0.1",31011);
  const auto viewer=bx72::endpoint("127.0.0.1",31012);
  bx72::Json message{{"v",1},{"type","control"},{"session","nav-owner"},{"seq",10},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,owner,1.99),bx72::PcInputResult::Observer);
  message["seq"]=11; message["claim_nonce"]=observers.find(owner)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,owner,2.0,true),bx72::PcInputResult::Owner);
  message["session"]="view-a"; message["seq"]=0xffffffffU;
  frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,2.05),bx72::PcInputResult::Observer);
  message["seq"]=0;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,2.1),bx72::PcInputResult::Observer);
  EXPECT_DOUBLE_EQ(lease.last,2.0);
  message["session"]="view-b"; message["seq"]=1;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,2.11),bx72::PcInputResult::Observer);
  EXPECT_EQ(observers.entries().size(),2U);
  EXPECT_EQ(lease.session,"nav-owner");
  message["session"]="view-a"; message["seq"]=1;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,viewer,2.111),bx72::PcInputResult::Rejected);
  EXPECT_EQ(observers.entries().back().session,"view-b");
  message["session"]="nav-owner"; message["seq"]=12;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,owner,2.12),bx72::PcInputResult::Owner);
  EXPECT_DOUBLE_EQ(lease.last,2.12);
  EXPECT_FALSE(lease.armed);
  EXPECT_TRUE(observers.expire(3.115));
  EXPECT_EQ(observers.entries().size(),1U);
  EXPECT_EQ(lease.session,"nav-owner");
}

TEST(PcMonitoring, AnIdleViewerAloneDoesNotCreateAControlOwner)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto peer=bx72::endpoint("127.0.0.1",31021);
  const bx72::Json idle{{"v",1},{"type","control"},{"session","old-web"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  const auto frame=bx72::parse_pc_control(idle);
  ASSERT_TRUE(frame);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,5.0),bx72::PcInputResult::Observer);
  EXPECT_TRUE(lease.session.empty());
  EXPECT_FALSE(lease.armed);
  ASSERT_EQ(observers.entries().size(),1U);
  EXPECT_EQ(observers.entries().front().session,"old-web");
}

TEST(PcMonitoring, FirstClaimAndExpiredSessionRequireARegisteredFreshNonce)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto peer=bx72::endpoint("127.0.0.1",31031);
  bx72::Json message{{"v",1},{"type","control"},{"session","one"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",true}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_TRUE(frame);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.0),bx72::PcInputResult::Rejected);
  message["take_control"]=false; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.0,true),bx72::PcInputResult::Rejected);
  EXPECT_TRUE(lease.session.empty());
  EXPECT_TRUE(observers.entries().empty());

  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.01),bx72::PcInputResult::Observer);
  const auto old_nonce=observers.find(peer)->claim_nonce;
  message["seq"]=2; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.02,true),bx72::PcInputResult::Rejected);
  message["take_control"]=true; message["claim"]=true; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.02),bx72::PcInputResult::Rejected);
  message["claim_nonce"]=old_nonce; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.03),bx72::PcInputResult::Owner);

  ASSERT_TRUE(observers.expire(2.04));
  message["seq"]=1000; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.05),bx72::PcInputResult::Rejected);
  message["take_control"]=false; message.erase("claim"); frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.05,true),bx72::PcInputResult::Rejected);
  message["seq"]=3; message.erase("claim_nonce"); frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.06),bx72::PcInputResult::Owner);
  const auto fresh_nonce=observers.find(peer)->claim_nonce;
  ASSERT_NE(old_nonce,fresh_nonce);
  message["seq"]=1000; message["take_control"]=true; message["claim"]=true; message["claim_nonce"]=old_nonce;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.07),bx72::PcInputResult::Rejected);
  message["seq"]=4; message["claim_nonce"]=fresh_nonce; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.08),bx72::PcInputResult::Owner);
  message["seq"]=1001; message["take_control"]=false; message.erase("claim"); message["claim_nonce"]=old_nonce;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.09,true),bx72::PcInputResult::Rejected);
  message["seq"]=5; message["claim_nonce"]=fresh_nonce; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.1,true),bx72::PcInputResult::Owner);
  message["seq"]=6; message.erase("claim_nonce"); frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,2.11,true,true),bx72::PcInputResult::Observer);
  EXPECT_EQ(lease.session,"one");
  EXPECT_EQ(lease.seq,5U);
}

TEST(PcMonitoring, ReacquisitionRequiresTheNewChallengeAndRejectsDelayedCommands)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto a=bx72::endpoint("127.0.0.1",31101), b=bx72::endpoint("127.0.0.1",31102);
  bx72::Json message{{"v",1},{"type","control"},{"session","a"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.0),bx72::PcInputResult::Observer);
  message["session"]="b"; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.01),bx72::PcInputResult::Observer);
  const auto a_before=observers.find(a)->claim_nonce;
  message["session"]="a"; message["seq"]=2; message["take_control"]=true; message["claim"]=true;
  message["claim_nonce"]=a_before;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.02),bx72::PcInputResult::Owner);
  message["session"]="b"; message["seq"]=2;
  message["claim_nonce"]=observers.find(b)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.03),bx72::PcInputResult::Owner);
  const auto a_after=observers.find(a)->claim_nonce;
  ASSERT_NE(a_before,a_after);
  message["session"]="a"; message["seq"]=1000; message["claim_nonce"]=a_before;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.04),bx72::PcInputResult::Rejected);
  EXPECT_EQ(lease.session,"b");
  message["seq"]=3; message["take_control"]=false; message.erase("claim"); message.erase("claim_nonce");
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.05,true,true),bx72::PcInputResult::Observer);
  EXPECT_EQ(lease.session,"b");
  message["seq"]=4; message["claim_nonce"]=a_after;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.06,true),bx72::PcInputResult::Rejected);
  message["claim_nonce"]=observers.find(a)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.06,true),bx72::PcInputResult::Owner);
  EXPECT_EQ(lease.session,"a");
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.07,true),bx72::PcInputResult::Rejected);
  message["seq"]=1001; message["claim_nonce"]=a_before; message["take_control"]=true;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.08),bx72::PcInputResult::Rejected);
}

TEST(PcMonitoring, CrossObserverStopInvalidatesDelayedStartAndEmergencyRelease)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto a=bx72::endpoint("127.0.0.1",31111), b=bx72::endpoint("127.0.0.1",31112);
  bx72::Json message{{"v",1},{"type","control"},{"session","a"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.0),bx72::PcInputResult::Observer);
  message["session"]="b"; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.01),bx72::PcInputResult::Observer);
  const auto before_stop=observers.find(a)->claim_nonce;
  message["session"]="a"; message["seq"]=2; message["claim_nonce"]=before_stop;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.02,true),bx72::PcInputResult::Owner);

  message["session"]="b"; message["seq"]=2; message.erase("claim_nonce");
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.03,true,true),bx72::PcInputResult::Observer);
  EXPECT_EQ(lease.session,"a");
  const auto after_stop=observers.find(a)->claim_nonce;
  ASSERT_NE(before_stop,after_stop);
  message["session"]="a"; message["seq"]=1000; message["claim_nonce"]=before_stop;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.04,true),bx72::PcInputResult::Rejected);
  message["seq"]=3; message["claim_nonce"]=after_stop; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.05,true),bx72::PcInputResult::Owner);

  message["session"]="b"; message["seq"]=3; message.erase("claim_nonce");
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.06,true,true),bx72::PcInputResult::Observer);
  message["session"]="a"; message["seq"]=1001; message["claim_nonce"]=after_stop;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.07,true),bx72::PcInputResult::Rejected);
  message["seq"]=4; message["claim_nonce"]=observers.find(a)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.08,true),bx72::PcInputResult::Owner);
}

TEST(PcMonitoring, CooperativeTaskCannotBeKeptAliveByAnotherOwner)
{
  bx72::PcLease lease;
  lease.session="owner-a"; lease.peer="127.0.0.1:31121"; lease.last=10.0;
  const auto lost = [&](bool ros, std::uint8_t mode, bool known, std::uint8_t flags, double now) {
    return bx72::cooperative_navigation_owner_lost(lease,ros,mode,known,flags,
                                                   "owner-a","127.0.0.1:31121",now);
  };
  EXPECT_FALSE(lost(false,2,true,0,10.1));
  lease.session="owner-b"; lease.peer="127.0.0.1:31122"; lease.last=10.1;
  EXPECT_TRUE(lost(false,2,true,0,10.11));
  EXPECT_FALSE(lost(false,2,true,1,10.11));  // allow_disconnect
  EXPECT_FALSE(lost(false,3,true,0,10.11));  // independent
  EXPECT_FALSE(lost(true,2,true,0,10.11));   // ROS-owned task
  lease.session="owner-a"; lease.peer="127.0.0.1:31121"; lease.last=10.0;
  EXPECT_TRUE(lost(false,2,true,0,10.21));
  EXPECT_TRUE(lost(false,2,false,0,10.1));
}

TEST(PcMonitoring, HeartbeatsCannotTakeBackControlOrRearmAfterStop)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto a=bx72::endpoint("127.0.0.1",31131), b=bx72::endpoint("127.0.0.1",31132);
  bx72::Json message{{"v",1},{"type","control"},{"session","a"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.0),bx72::PcInputResult::Observer);
  message["session"]="b"; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.01),bx72::PcInputResult::Observer);
  message["session"]="a"; message["seq"]=2; message["take_control"]=true; message["claim"]=true;
  message["claim_nonce"]=observers.find(a)->claim_nonce; frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.02),bx72::PcInputResult::Owner);
  message["session"]="b"; message["claim_nonce"]=observers.find(b)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.03),bx72::PcInputResult::Owner);
  message["session"]="a"; message["seq"]=3; message.erase("claim");
  message["claim_nonce"]=observers.find(a)->claim_nonce; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,a,1.04),bx72::PcInputResult::Rejected);
  EXPECT_EQ(lease.session,"b");
  message["session"]="b"; message["seq"]=3; message["take_control"]=false; message.erase("claim_nonce");
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.05,true,true),bx72::PcInputResult::Observer);
  lease.disarm();
  message["seq"]=4; message["take_control"]=true; message["claim_nonce"]=observers.find(b)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.06),bx72::PcInputResult::Rejected);
  message["claim"]=true; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,b,1.07),bx72::PcInputResult::Owner);
}

TEST(PcMonitoring, ExpiredLeaseNeedsAnExplicitClaimBeforeRearming)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto peer=bx72::endpoint("127.0.0.1",31141);
  bx72::Json message{{"v",1},{"type","control"},{"session","a"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.0),bx72::PcInputResult::Observer);
  message["seq"]=2; message["take_control"]=true; message["claim"]=true;
  message["claim_nonce"]=observers.find(peer)->claim_nonce;
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.01),bx72::PcInputResult::Owner);
  message["seq"]=3; message.erase("claim"); frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.22),bx72::PcInputResult::Rejected);
  EXPECT_FALSE(lease.armed);
  message["claim"]=true; frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.23),bx72::PcInputResult::Owner);
}

TEST(PcMonitoring, RetiredSessionCanStopWithoutReplacingTheCurrentObserver)
{
  bx72::PcLease lease;
  bx72::PcObservers observers;
  const auto peer=bx72::endpoint("127.0.0.1",31103);
  bx72::Json message{{"v",1},{"type","control"},{"session","old"},{"seq",1},
    {"vx",0},{"vy",0},{"wz",0},{"auto",false},{"take_control",false}};
  auto frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.0),bx72::PcInputResult::Observer);
  message["session"]="new";
  frame=bx72::parse_pc_control(message);
  ASSERT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.01),bx72::PcInputResult::Observer);
  message["session"]="old"; message["seq"]=2;
  frame=bx72::parse_pc_control(message);
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.02,true,true),bx72::PcInputResult::Observer);
  EXPECT_EQ(observers.find(peer)->session,"new");
  EXPECT_EQ(observers.find(peer)->seq,1U);
  EXPECT_TRUE(lease.session.empty());
  EXPECT_EQ(bx72::accept_pc_input(lease,observers,*frame,peer,1.03,true),bx72::PcInputResult::Rejected);
}

TEST(Yuv422Rtp, DropsFrameAfterSequenceGapAndRecoversOnNextFrame)
{
  bx72::RtpFrames frames(4, 1);
  EXPECT_FALSE(frames.feed(
      rtp_packet(bx72::video_payload_type, 10, 5, false, {16, 128, 16, 128})).has_value());
  EXPECT_FALSE(frames.feed(
      rtp_packet(bx72::video_payload_type, 12, 5, true, {16, 128, 16, 128})).has_value());
  EXPECT_EQ(frames.complete, 0U);
  EXPECT_EQ(frames.dropped, 1U);

  const auto recovered = frames.feed(
    rtp_packet(bx72::video_payload_type, 13, 6, true,
      {16, 128, 16, 128, 16, 128, 16, 128}));
  ASSERT_TRUE(recovered.has_value());
  EXPECT_EQ(recovered->size(), 8U);
  EXPECT_EQ(frames.complete, 1U);
}

}  // namespace
