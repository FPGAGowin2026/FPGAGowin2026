#include <chrono>
#include <vector>

#include <gtest/gtest.h>

#include "BYTETracker.h"

#define CAR_MISSION_VISION_NODE_NO_MAIN
#include "../src/vision_node.cpp"

namespace {

Object make_detection(float x, int detection_index)
{
  Object detection;
  detection.rect = cv::Rect2f(x, 20.0F, 30.0F, 40.0F);
  detection.label = 0;
  detection.prob = 0.95F;
  detection.detection_index = detection_index;
  return detection;
}

TEST(ByteTrackObservation, KeepsMatchedInputAndDoesNotReturnLostPrediction)
{
  BYTETracker tracker(10, 30);
  auto tracks = tracker.update({make_detection(10.0F, 4)});
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().frame_id, tracker.current_frame_id());
  EXPECT_EQ(tracks.front().detection_index, 4);
  ASSERT_EQ(tracks.front().observed_tlwh.size(), 4U);
  EXPECT_FLOAT_EQ(tracks.front().observed_tlwh[0], 10.0F);
  EXPECT_FLOAT_EQ(tracks.front().observed_tlwh[1], 20.0F);
  EXPECT_FLOAT_EQ(tracks.front().observed_tlwh[2], 30.0F);
  EXPECT_FLOAT_EQ(tracks.front().observed_tlwh[3], 40.0F);

  EXPECT_TRUE(tracker.update({}).empty());

  tracks = tracker.update({make_detection(10.0F, 0)});
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().frame_id, tracker.current_frame_id());
  EXPECT_EQ(tracks.front().detection_index, 0);
}

TEST(ByteTrackObservation, ThresholdsAndClassIdentityAreRespected)
{
  auto low = make_detection(10.0F, 0); low.prob = 0.3F;
  BYTETracker defaults(10, 30);
  EXPECT_TRUE(defaults.update({low}).empty());
  BYTETracker configured(10, 30, 0.2F, 0.25F);
  const auto first = configured.update({low});
  ASSERT_EQ(first.size(), 1U);
  const int first_id = first.front().track_id;
  low.label = 1;
  auto changed = configured.update({low});
  for (const auto & track : changed) EXPECT_NE(track.track_id, first_id);
  changed = configured.update({low});
  ASSERT_EQ(changed.size(), 1U);
  EXPECT_NE(changed.front().track_id, first_id);
  EXPECT_EQ(changed.front().class_id, 1);
}

TEST(VisionPreprocess, ConvertsBgrToRgbAndReversesLetterbox)
{
  cv::Mat image(208, 416, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<float> input(3U * 416U * 416U, 0.0F);
  const auto letterbox = car_mission::preprocess_to_nchw(image, input);
  EXPECT_EQ(letterbox.left, 0);
  EXPECT_EQ(letterbox.top, 104);
  EXPECT_FLOAT_EQ(letterbox.scale_x, 1.0F);
  EXPECT_FLOAT_EQ(letterbox.scale_y, 1.0F);
  EXPECT_FLOAT_EQ(input[0], 114.0F / 255.0F);
  EXPECT_FLOAT_EQ(input[104U * 416U], 30.0F / 255.0F);
  EXPECT_FLOAT_EQ(input[416U * 416U + 104U * 416U], 20.0F / 255.0F);
  EXPECT_FLOAT_EQ(input[2U * 416U * 416U + 104U * 416U], 10.0F / 255.0F);

  const auto restored = car_mission::restore_xyxy(20.0F, 124.0F, 120.0F, 204.0F,
      letterbox, image.cols, image.rows);
  EXPECT_FLOAT_EQ(restored.x, 20.0F);
  EXPECT_FLOAT_EQ(restored.y, 20.0F);
  EXPECT_FLOAT_EQ(restored.width, 100.0F);
  EXPECT_FLOAT_EQ(restored.height, 80.0F);
}

TEST(VisionPreprocess, ResizesFpgaFrameAndReversesRoundedLetterboxScale)
{
  cv::Mat image(480, 800, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<float> input(3U * 416U * 416U, 0.0F);
  const auto letterbox = car_mission::preprocess_to_nchw(image, input);
  EXPECT_EQ(letterbox.left, 0);
  EXPECT_EQ(letterbox.top, 83);
  EXPECT_FLOAT_EQ(letterbox.scale_x, 416.0F / 800.0F);
  EXPECT_FLOAT_EQ(letterbox.scale_y, 250.0F / 480.0F);

  const auto restored = car_mission::restore_xyxy(
    52.0F, 83.0F + 100.0F * letterbox.scale_y,
    312.0F, 83.0F + 300.0F * letterbox.scale_y,
    letterbox, image.cols, image.rows);
  EXPECT_NEAR(restored.x, 100.0F, 1.0e-4F);
  EXPECT_NEAR(restored.y, 100.0F, 1.0e-4F);
  EXPECT_NEAR(restored.width, 500.0F, 1.0e-4F);
  EXPECT_NEAR(restored.height, 200.0F, 1.0e-4F);
}

TEST(VisionOutputLayout, AcceptsRowsAndRawCocoLayout)
{
  nvinfer1::Dims rows{};
  rows.nbDims = 3;
  rows.d[0] = 1;
  rows.d[1] = 300;
  rows.d[2] = 6;
  EXPECT_EQ(car_mission::output_rows_for_format("rows_xyxy_score_class", rows), 300U);

  nvinfer1::Dims raw{};
  raw.nbDims = 3;
  raw.d[0] = 1;
  raw.d[1] = 84;
  raw.d[2] = 3549;
  EXPECT_EQ(car_mission::output_rows_for_format("rows_xyxy_score_class", raw), 3549U);
  EXPECT_THROW(car_mission::output_rows_for_format("raw_yolo", rows), std::runtime_error);

  nvinfer1::Dims unbatched_rows{};
  unbatched_rows.nbDims = 2;
  unbatched_rows.d[0] = 300;
  unbatched_rows.d[1] = 6;
  EXPECT_EQ(
    car_mission::output_rows_for_format("rows_xyxy_score_class", unbatched_rows), 300U);
}

TEST(VisionInputStamp, RejectsZeroFutureAndStaleTimestamps)
{
  EXPECT_EQ(car_mission::stamp_nanoseconds(-1, 0), 0);
  EXPECT_EQ(car_mission::stamp_nanoseconds(1, 1'000'000'000U), 0);
  EXPECT_FALSE(car_mission::source_stamp_is_fresh(0, 2'000'000'000LL, 0.5));
  EXPECT_FALSE(car_mission::source_stamp_is_fresh(2'100'000'000LL, 2'000'000'000LL, 0.5));
  EXPECT_FALSE(car_mission::source_stamp_is_fresh(1'000'000'000LL, 2'000'000'001LL, 0.5));
  EXPECT_TRUE(car_mission::source_stamp_is_fresh(1'500'000'000LL, 2'000'000'000LL, 0.5));
}

TEST(VisionInputStamp, AcceptsOnlyStrictlyIncreasingSourceTimestamps)
{
  EXPECT_TRUE(car_mission::source_stamp_is_newer(2'000'000'001LL, 2'000'000'000LL));
  EXPECT_FALSE(car_mission::source_stamp_is_newer(2'000'000'000LL, 2'000'000'000LL));
  EXPECT_FALSE(car_mission::source_stamp_is_newer(1'999'999'999LL, 2'000'000'000LL));
}

TEST(VisionInput, RequiresFrameIdAndCompleteSupportedImageLayout)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera";
  image.width = 1;
  image.height = 1;
  image.encoding = sensor_msgs::image_encodings::BGR8;
  image.step = 3;
  image.data.resize(3);
  EXPECT_TRUE(car_mission::image_message_is_valid(image));

  image.header.frame_id.clear();
  EXPECT_FALSE(car_mission::image_message_is_valid(image));
  image.header.frame_id = "camera";

  image.step = 2;
  EXPECT_FALSE(car_mission::image_message_is_valid(image));
  image.step = 3;
  image.data.resize(2);
  EXPECT_FALSE(car_mission::image_message_is_valid(image));
  image.data.resize(3);
  image.encoding = "yuv422";
  EXPECT_FALSE(car_mission::image_message_is_valid(image));
}

TEST(VisionInputFreshness, ExpiresBySteadyReceiptAge)
{
  using Clock = std::chrono::steady_clock;
  const Clock::time_point received{};
  EXPECT_TRUE(car_mission::steady_receipt_is_fresh(
      received, received + std::chrono::milliseconds(500), 0.5));
  EXPECT_FALSE(car_mission::steady_receipt_is_fresh(
      received, received + std::chrono::milliseconds(501), 0.5));
  EXPECT_FALSE(car_mission::steady_receipt_is_fresh(
      received, received - std::chrono::milliseconds(1), 0.5));
}

}  // namespace
