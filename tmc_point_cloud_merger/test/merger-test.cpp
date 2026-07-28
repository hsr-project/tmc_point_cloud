/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @file     merger-test.cpp
/// @brief    Test for MergedCloudPublisher class
/// @author   Fukukazu Kawata

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_point_cloud_merger/merger.hpp>
#include <tmc_point_cloud_merger/param.hpp>

#include "utils.hpp"

namespace {
// Expected point_step for the converted Message type (=byte size representing one point)
constexpr uint8_t kPointStep = 16;
}  // anonymous namespace

namespace tmc_point_cloud_merger {
class MergedCloudPublisherTest : public testing::Test {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Test node creation
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    test_node_ = std::make_shared<rclcpp::Node>("merger_test", options);
    LoadParameterFromYaml(
      test_node_,
      ament_index_cpp::get_package_share_directory("tmc_point_cloud_merger") + "/test/params/",
      "merger_test.yaml");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

 protected:
  rclcpp::Node::SharedPtr test_node_;
  MergedCloudPublisher::SharedPtr merged_cloud_publisher_;
};

TEST_F(MergedCloudPublisherTest, CreateFunctionOK) {
  // setup
  std::map<std::string, rclcpp::Parameter> params;
  ASSERT_TRUE(GetGroupParam(test_node_, "valid_param", params));
  auto init = [&](std::map<std::string, rclcpp::Parameter>& p) {
      merged_cloud_publisher_ = MergedCloudPublisher::Create(test_node_->shared_from_this(), p);
    };

  // exercise
  ASSERT_NO_THROW(init(params));
  auto topics = GetPublishedNodeTopics(test_node_);

  // verify
  EXPECT_TRUE(merged_cloud_publisher_ != nullptr);
  ASSERT_EQ(3, static_cast<int>(topics.size()));
  EXPECT_STREQ("/merged_cloud", topics[0].c_str());
  EXPECT_STREQ("/parameter_events", topics[1].c_str());
  EXPECT_STREQ("/rosout", topics[2].c_str());
}

class CreateFunctionTestNG
  : public MergedCloudPublisherTest, public testing::WithParamInterface<std::string> {};

INSTANTIATE_TEST_CASE_P(
  CreateFunctionNG,
  CreateFunctionTestNG,
  testing::Values(
    "topic_name_not_found", "topic_name_empty", "topic_name_invalid_type",
    "frame_id_not_found", "frame_id_empty", "frame_id_invalid_type"));

TEST_P(CreateFunctionTestNG, CreateFunctionNG) {
  // setup
  const std::string group_name = GetParam();
  std::map<std::string, rclcpp::Parameter> params;
  EXPECT_TRUE(GetGroupParam(test_node_, group_name, params));

  auto init = [&](std::map<std::string, rclcpp::Parameter> & p) {
      merged_cloud_publisher_ = MergedCloudPublisher::Create(test_node_->shared_from_this(), p);
    };

  // exercise and verify
  EXPECT_THROW(init(params), std::invalid_argument) << "No throws by using: " << group_name.c_str();
}

TEST_F(MergedCloudPublisherTest, ConstructorOK) {
  // exercise
  ASSERT_NO_THROW(merged_cloud_publisher_.reset(new MergedCloudPublisher(
    test_node_, "some_topic", "some_link")));

  auto topics = GetPublishedNodeTopics(test_node_);

  // verify
  ASSERT_TRUE(merged_cloud_publisher_ != nullptr);
  ASSERT_EQ(3, static_cast<int>(topics.size()));
  EXPECT_STREQ("/parameter_events", topics[0].c_str());
  EXPECT_STREQ("/rosout", topics[1].c_str());
  EXPECT_STREQ("/some_topic", topics[2].c_str());
}

struct ConstructorTestNGParam {
  std::string topic_name;
  std::string frame_id;
};

class ConstructorTestNG
  : public MergedCloudPublisherTest, public testing::WithParamInterface<ConstructorTestNGParam> {};

INSTANTIATE_TEST_CASE_P(
  ConstructorNG,
  ConstructorTestNG,
  testing::Values(
    ConstructorTestNGParam {"", "some_link"},
    ConstructorTestNGParam {"some_topic", ""}));

TEST_P(ConstructorTestNG, ConstructorNG) {
  // setup
  ConstructorTestNGParam p = GetParam();

  // exercise and verify
  ASSERT_THROW(merged_cloud_publisher_.reset(new MergedCloudPublisher(
    test_node_->shared_from_this(), p.topic_name, p.frame_id)), std::invalid_argument)
    << "No throws by using: " << p.topic_name.c_str() << " and " << p.frame_id;
}

class BehaviorTest : public testing::Test {
 public:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Test node creation
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    test_node_ = std::make_shared<rclcpp::Node>("merger_test", options);
    LoadParameterFromYaml(
      test_node_,
      ament_index_cpp::get_package_share_directory("tmc_point_cloud_merger") + "/test/params/",
      "merger_test.yaml");

    // MergedCloudPublisher creation
    std::map<std::string, rclcpp::Parameter> params;
    ASSERT_TRUE(GetGroupParam(test_node_, "valid_param", params));
    merged_cloud_publisher_ = MergedCloudPublisher::Create(test_node_->shared_from_this(), params);

    // CacheSubscriber creation
    cache_.reset(new CacheSubscriber<Message>(
      test_node_->shared_from_this(), params["topic_name"].get_value<std::string>()));
    cache_->StartCaching();
    ASSERT_TRUE(WaitUntil([&]() {return cache_->IsMessageAvailable();}, 3.0));

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
    SendStaticTransforms();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  void SendStaticTransforms() {
    // map_to_s0
    SendStaticTransform("1.0 1.0 0.0 -0.7853981633974483 0.0 0.0 map s0", tf_broadcaster_);
    // map_to_s1
    SendStaticTransform("1.0 0.0 0.0 0.0 0.0 0.0 map s1", tf_broadcaster_);
    // map_to_s2
    SendStaticTransform("1.0 -1.0 0.0 0.7853981633974483 0.0 0.0 map s2", tf_broadcaster_);
  }

 protected:
  rclcpp::Node::SharedPtr test_node_;
  MergedCloudPublisher::SharedPtr merged_cloud_publisher_;
  CacheSubscriber<Message>::SharedPtr cache_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

struct MergeBehaviorTestParam {
  MergedCloudPublisher::Subject subject;
  std::vector<Point> expected_ps;
};

class MergeBehaviorTest
  : public BehaviorTest, public testing::WithParamInterface<MergeBehaviorTestParam> {};

INSTANTIATE_TEST_CASE_P(
  CanMergeCorrectly,
  MergeBehaviorTest,
  testing::Values(
    // Single item - XYZ
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0}}},
    // Single item - XYZRGB
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0, 0, 128, 255)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0}}},
    // Single item - XYZI
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0, 0.6)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0}}},
    // Combination of 3 items - XYZ, XYZ, XYZ
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0)))},
      {"hoge_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s1", 1.0, 0.0, 0.0)))},
      {"some_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s2", 1.0, 0.0, 0.0)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0},
      Point{2.0, 0.0, 0.0},
      Point{1.7071067811865476, -0.2928932188134524, 0.0}}},
    // Combination of 3 items - XYZRGB, XYZRGB, XYZRGB
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"hoge_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s1", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"some_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s2", 1.0, 0.0, 0.0, 0, 128, 255)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0},
      Point{2.0, 0.0, 0.0},
      Point{1.7071067811865476, -0.2928932188134524, 0.0}}},
    // Combination of 3 items - XYZ, XYZRGB, XYZRGB
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0)))},
      {"hoge_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s1", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"some_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s2", 1.0, 0.0, 0.0, 0, 128, 255)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0},
      Point{2.0, 0.0, 0.0},
      Point{1.7071067811865476, -0.2928932188134524, 0.0}}},
    // Combination of 3 items - XYZRGB, XYZ, XYZRGB
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"hoge_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s1", 1.0, 0.0, 0.0)))},
      {"some_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s2", 1.0, 0.0, 0.0, 0, 128, 255)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0},
      Point{2.0, 0.0, 0.0},
      Point{1.7071067811865476, -0.2928932188134524, 0.0}}},
    // Combination of 3 items - XYZRGB, XYZRGB, XYZ
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"hoge_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s1", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"some_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s2", 1.0, 0.0, 0.0)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0},
      Point{2.0, 0.0, 0.0},
      Point{1.7071067811865476, -0.2928932188134524, 0.0}}},
    // Combination of 3 items - XYZRGB, XYZI, XYZ
    MergeBehaviorTestParam{
    {{"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s0", 1.0, 0.0, 0.0, 0, 128, 255)))},
      {"hoge_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s1", 1.0, 0.0, 0.0, 0.5)))},
      {"some_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
        rclcpp::Clock(RCL_ROS_TIME).now(), "s2", 1.0, 0.0, 0.0)))}},
    {Point{1.7071067811865476, 0.2928932188134524, 0.0},
      Point{2.0, 0.0, 0.0},
      Point{1.7071067811865476, -0.2928932188134524, 0.0}}}));

TEST_P(MergeBehaviorTest, CanMergeCorrectly) {
  // setup
  const MergedCloudPublisher::Subject subject = GetParam().subject;
  rclcpp::Time expected_time_stamp(0, 0, RCL_ROS_TIME);
  for (auto it = subject.begin(); it != subject.end(); ++it) {
    auto stamp = it->second->header.stamp;
    rclcpp::Time header_time(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
    if (header_time > expected_time_stamp) {
      expected_time_stamp = header_time;
    }
  }

  // exercise
  merged_cloud_publisher_->MergeAndPublish(GetParam().subject);
  rclcpp::spin_some(test_node_);
  ASSERT_TRUE(WaitUntil([&]() {return cache_->GetQueueLength() > 0;}, 3.0));
  Message msg = cache_->GetLatestMessage();

  // verify
  pcl::PointCloud<Point> p;
  pcl::fromROSMsg(msg, p);
  rclcpp::Time message_time = msg.header.stamp;
  EXPECT_EQ(expected_time_stamp, message_time);
  EXPECT_STREQ("map", msg.header.frame_id.c_str());
  ASSERT_FALSE(p.points.empty());
  ASSERT_EQ(GetParam().expected_ps.size(), p.points.size());
  EXPECT_EQ(msg.point_step, kPointStep);
  bool has_only_xyz_field = false;
  std::vector<sensor_msgs::msg::PointField> non_xyz_fields;
  std::copy_if(
    msg.fields.begin(), msg.fields.end(),
    std::back_inserter(non_xyz_fields),
    [](const sensor_msgs::msg::PointField& f) {return f.name != "x" && f.name != "y" && f.name != "z";});
  if (non_xyz_fields.empty()) {
    has_only_xyz_field = true;
  }
  EXPECT_TRUE(has_only_xyz_field);
  for (size_t i = 0; i < GetParam().expected_ps.size(); ++i) {
    EXPECT_NEAR(GetParam().expected_ps[i].x, p.points[i].x, 1e-6)
      << "actual point[" << i << "]: (" << p[i].x << ", " << p[i].y << ", " << p[i].z << ")";
    EXPECT_NEAR(GetParam().expected_ps[i].y, p.points[i].y, 1e-6)
      << "actual point[" << i << "]: (" << p[i].x << ", " << p[i].y << ", " << p[i].z << ")";
    EXPECT_NEAR(GetParam().expected_ps[i].z, p.points[i].z, 1e-6)
      << "actual point[" << i << "]: (" << p[i].x << ", " << p[i].y << ", " << p[i].z << ")";
  }
}

TEST_F(BehaviorTest, CanIgnoreEmptyFrame) {
  // setup
  MergedCloudPublisher::Subject subject;

  // exercise and verify
  merged_cloud_publisher_->MergeAndPublish(subject);
  rclcpp::spin_some(test_node_);
  ASSERT_FALSE(WaitUntil([&]() {return cache_->GetQueueLength() > 0;}, 0.1));
}

TEST_F(BehaviorTest, CannotTransformByUsingInvalidFrameID) {
  // setup
  MergedCloudPublisher::Subject subject {
    {"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Clock(RCL_ROS_TIME).now(), "invalid_id", 1.0, 0.0, 0.0)))}};

  // exercise and verify
  merged_cloud_publisher_->MergeAndPublish(subject);
  rclcpp::spin_some(test_node_);
  ASSERT_FALSE(WaitUntil([&]() {return cache_->GetQueueLength() > 0;}, 0.1));
}

TEST_F(BehaviorTest, CannotTransformByUsingEmptyMessage) {
  // setup
  MergedCloudPublisher::Subject subject {{"any_topic_name", Message::ConstSharedPtr()}};

  // exercise and verify
  merged_cloud_publisher_->MergeAndPublish(subject);
  rclcpp::spin_some(test_node_);
  ASSERT_FALSE(WaitUntil([&]() {return cache_->GetQueueLength() > 0;}, 0.1));
}

}  // namespace tmc_point_cloud_merger
