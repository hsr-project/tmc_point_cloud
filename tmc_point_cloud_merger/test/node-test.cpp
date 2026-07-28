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
/// @file     node-test.cpp
/// @brief    Test for Node class
/// @author   Fukukazu Kawata

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tmc_point_cloud_merger/point_cloud_merger_node.hpp>
#include <tmc_point_cloud_merger/type.hpp>

#include "utils.hpp"

namespace {
const double kEpsilon = 1e-6;
}  // anonymous namespace

namespace tmc_point_cloud_merger {

struct TopicNameAndFrameID {
  std::string topic_name;
  std::string frame_id;
};

class NodeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Generate test node
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    test_node_ = std::make_shared<rclcpp::Node>("point_cloud_merger_node_test", options);
    LoadParameterFromYaml(
      test_node_,
      ament_index_cpp::get_package_share_directory("tmc_point_cloud_merger") + "/test/params/",
      "node_test.yaml");

    // Publish static TF
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
    SendStaticTransforms();
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_ && executor_thread_->joinable()) {
      executor_thread_->join();
    }

    rclcpp::shutdown();
  }

  void NodesLaunch() {
    rclcpp::NodeOptions node_options;
    GetTestNodeOptions("valid_param", &node_options);
    point_cloud_merger_node_ = std::make_shared<PointCloudMergerNode>(node_options);
    EXPECT_NO_THROW(point_cloud_merger_node_->Init());

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
    executor_->add_node(point_cloud_merger_node_);
    executor_thread_ = std::make_shared<std::thread>([this]() {
      executor_->spin();
    });
  }

  void GetNodeGroupParam(
    const rclcpp::Node::SharedPtr& node, const std::string& group_name,
    std::vector<rclcpp::Parameter>* parameters) {
    // Retrieve test parameters
    std::map<std::string, rclcpp::Parameter> params;
    ASSERT_TRUE(GetGroupParam(node, group_name, params));

    // Convert from map to vector
    for (const auto& param : params) {
      // param.first : topic name (inputs.input1.topic_name)
      // param.second: topic name (valid_param.inputs.input1.topic_name), topic value
      rclcpp::Parameter new_param(param.first, param.second.get_parameter_value());
      parameters->push_back(new_param);
    }
  }

  void GetTestNodeOptions(const std::string& group_name, rclcpp::NodeOptions* options) {
    std::vector<rclcpp::Parameter> parameters;
    GetNodeGroupParam(test_node_->shared_from_this(), group_name, &parameters);
    options->automatically_declare_parameters_from_overrides(true);
    options->parameter_overrides(parameters);
  }

  void GeneratePublishers(
    const rclcpp::Node::SharedPtr node,
    std::vector<CyclicMessagePublisher<Message>::SharedPtr>& pubs) {
    std::vector<CyclicMessagePublisher<Message>::SharedPtr>().swap(pubs);
    pubs.resize(3, nullptr);

    std::vector<TopicNameAndFrameID> pub_settings {
      TopicNameAndFrameID {"/camera0/points", "s0"},
      TopicNameAndFrameID {"/camera1/points", "s1"},
      TopicNameAndFrameID {"/camera2/points", "s2"}};

    pcl::PointCloud<Point> p;
    p.push_back(Point(1.0, 0.0, 0.0));
    Message pmsg;
    pcl::toROSMsg<Point>(p, pmsg);

    for (int i = 0; i < 3; ++i) {
      pubs[i].reset(new CyclicMessagePublisher<Message>(
          node, pub_settings[i].topic_name, pub_settings[i].frame_id));
      pubs[i]->set_msg(pmsg);
    }
  }

  void SendStaticTransforms() {
    // map_to_s0
    SendStaticTransform("1.0 1.0 0.0 -0.7853981633974483 0.0 0.0 map s0", tf_broadcaster_);
    // map_to_s1
    SendStaticTransform("1.0 0.0 0.0 0.0 0.0 0.0 map s1", tf_broadcaster_);
    // map_to_s2
    SendStaticTransform("1.0 -1.0 0.0 0.7853981633974483 0.0 0.0 map s2", tf_broadcaster_);
  }

  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<PointCloudMergerNode> point_cloud_merger_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> executor_thread_;
};

// Load succeeds with valid parameters
TEST_F(NodeTest, CanLoaded) {
  // setup
  rclcpp::NodeOptions options;
  GetTestNodeOptions("valid_param", &options);
  point_cloud_merger_node_ = std::make_shared<PointCloudMergerNode>(options);

  // exercise and verify
  EXPECT_NO_THROW(point_cloud_merger_node_->Init());
}

// Constructor test parameters
class InitNodeTest
  : public NodeTest, public testing::WithParamInterface<std::string> {};

INSTANTIATE_TEST_CASE_P(
  CannotLoadedWithLackingParameter,
  InitNodeTest,
  testing::Values(
    "no_input", "no_trigger", "no_output", "include_invalid_type"
  )
);

TEST_P(InitNodeTest, CannotLoadedWithLackingParameter) {
  // setup
  rclcpp::NodeOptions options;
  GetTestNodeOptions(GetParam(), &options);
  point_cloud_merger_node_ = std::make_shared<PointCloudMergerNode>(options);

  // exercise and verify
  ASSERT_THROW(point_cloud_merger_node_->Init(), std::invalid_argument);
}

// Able to subscribe as configured
TEST_F(NodeTest, CanInputVariousNumberOfMessages) {
  // setup
  rclcpp::NodeOptions options;
  GetTestNodeOptions("valid_param", &options);
  point_cloud_merger_node_ = std::make_shared<PointCloudMergerNode>(options);
  EXPECT_NO_THROW(point_cloud_merger_node_->Init());

  // exercise
  auto topics = GetSubscribedNodeTopics(point_cloud_merger_node_);

  // verify
  ASSERT_EQ(4, static_cast<int>(topics.size()));
  EXPECT_STREQ("/camera0/points", topics[0].c_str());
  EXPECT_STREQ("/camera1/points", topics[1].c_str());
  EXPECT_STREQ("/camera2/points", topics[2].c_str());
  EXPECT_STREQ("/parameter_events", topics[3].c_str());
}

// Can publish properly merged results, and the values are accurate
TEST_F(NodeTest, CanMergeCloudsWithCorrectTransformation) {
  // Test data transmitter
  std::vector<CyclicMessagePublisher<Message>::SharedPtr> pubs;
  GeneratePublishers(test_node_->shared_from_this(), pubs);

  // Output buffer
  auto queue = CacheSubscriber<Message>(test_node_->shared_from_this(), "merged_cloud");

  // Execute Executor
  NodesLaunch();

  // Expected value
  std::vector<Point> expected_points {
    Point {1.7071067811865476, 0.2928932188134524, 0.0},
    Point {2.0, 0.0, 0.0},
    Point {1.7071067811865476, -0.2928932188134524, 0.0}};

  // exercise
  // Launch three publishers and input to the target
  // Obtain the latest value when sufficient data is collected
  ASSERT_TRUE(WaitUntil([&]() {
      for (auto pub : pubs) {
        if (!pub->IsSubscribed()) {return false;}
      }
      return true;
    }, 3.0));

  ASSERT_EQ(queue.GetQueueLength(), 0);
  queue.StartCaching();

  ASSERT_TRUE(WaitUntil([&]() {return queue.IsMessageAvailable();}, 0.5));
  for (auto pub : pubs) {
    pub->StartPublishing<DummySensorTimeForward>(100.0);
  }

  ASSERT_TRUE(WaitUntil([&]() {return queue.GetQueueLength() >= 10;}, 3.0));

  for (auto pub : pubs) {
    pub->StopPublishing();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  Message merged_msg = queue.GetLatestMessage();
  pcl::PointCloud<Point> merged_p;
  pcl::fromROSMsg<Point>(merged_msg, merged_p);

  // verify
  EXPECT_EQ(merged_msg.header.frame_id, "map");
  EXPECT_EQ(merged_p.points.size(), 3);  // Input is designed to result in 3 points
  for (int i = 0; i < 3; ++i) {  // Processed sequentially from camera0, so the order of points is determined accordingly
    EXPECT_NEAR(merged_p.points[i].x, expected_points[i].x, kEpsilon);
    EXPECT_NEAR(merged_p.points[i].y, expected_points[i].y, kEpsilon);
    EXPECT_NEAR(merged_p.points[i].z, expected_points[i].z, kEpsilon);
  }
}

// Stop one message midway
// Confirm that it is excluded from the results
TEST_F(NodeTest, CanDropStalledDataFromTargets) {
  // setup
  std::vector<CyclicMessagePublisher<Message>::SharedPtr> pubs;
  GeneratePublishers(test_node_->shared_from_this(), pubs);
  auto queue = CacheSubscriber<Message>(test_node_->shared_from_this(), "merged_cloud");
  NodesLaunch();
  std::vector<Point> expected_points {
    Point {1.7071067811865476, 0.2928932188134524, 0.0},
    Point {1.7071067811865476, -0.2928932188134524, 0.0}};

  // exercise
  // Initially input data to the target with three publishers
  // After confirming that the merged data becomes three, stop the output of one publisher
  // Wait until the merged data becomes two, then obtain the latest data at that time
  ASSERT_TRUE(WaitUntil([&]() {
      for (auto pub : pubs) {
        if (!pub->IsSubscribed()) {return false;}
      }
      return true;
    }, 3.0));
  ASSERT_EQ(queue.GetQueueLength(), 0);

  queue.StartCaching();
  ASSERT_TRUE(WaitUntil([&]() {return queue.IsMessageAvailable();}, 0.5));
  for (auto pub : pubs) {
    pub->StartPublishing<DummySensorTimeForward>(100.0);
  }
  ASSERT_TRUE(WaitUntil([&]() {
        if (queue.GetQueueLength() <= 10) {return false;}
        Message msg = queue.GetLatestMessage();
        pcl::PointCloud<Point> p;
        pcl::fromROSMsg<Point>(msg, p);
        return p.points.size() == 3;
      }, 3.0));

  pubs[1]->StopPublishing();
  ASSERT_TRUE(WaitUntil([&]() {
        if (queue.GetQueueLength() <= 10) {return false;}
        Message msg = queue.GetLatestMessage();
        pcl::PointCloud<Point> p;
        pcl::fromROSMsg<Point>(msg, p);
        return p.points.size() == 2;
      }, 3.0));

  for (auto pub : pubs) {
    pub->StopPublishing();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  queue.StopCaching();

  Message merged_msg = queue.GetLatestMessage();
  pcl::PointCloud<Point> merged_p;
  pcl::fromROSMsg<Point>(merged_msg, merged_p);

  // verify
  EXPECT_EQ(merged_msg.header.frame_id, "map");
  EXPECT_EQ(merged_p.points.size(), 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(merged_p.points[i].x, expected_points[i].x, kEpsilon);
    EXPECT_NEAR(merged_p.points[i].y, expected_points[i].y, kEpsilon);
    EXPECT_NEAR(merged_p.points[i].z, expected_points[i].z, kEpsilon);
  }
}

// Confirm that even if input is dropped once, it is properly merged when resumed
TEST_F(NodeTest, CanRestoreDroppedNamedTopicWhenNewOneComes) {
  // setup
  std::vector<CyclicMessagePublisher<Message>::SharedPtr> pubs;
  GeneratePublishers(test_node_->shared_from_this(), pubs);
  auto queue = CacheSubscriber<Message>(test_node_->shared_from_this(), "merged_cloud");
  NodesLaunch();

  // Position observed from the map for each point
  std::vector<Point> expected_points {
    Point {1.7071067811865476, 0.2928932188134524, 0.0},
    Point {2.0, 0.0, 0.0},
    Point {1.7071067811865476, -0.2928932188134524, 0.0}};

  // exercise
  // Initially input data to the target with three publishers
  // After confirming that the merged data becomes three, stop the output of one publisher
  // Wait until the merged data becomes two, then resume publishing with three publishers
  // Obtain the latest value when sufficient data is collected
  ASSERT_TRUE(WaitUntil([&]() {
      for (auto pub : pubs) {
        if (!pub->IsSubscribed()) {return false;}
      }
      return true;
    }, 3.0));
  ASSERT_EQ(queue.GetQueueLength(), 0);
  queue.StartCaching();
  ASSERT_TRUE(WaitUntil([&]() {return queue.IsMessageAvailable();}, 0.5));

  for (auto pub : pubs) {
    pub->StartPublishing<DummySensorTimeForward>(100.0);
  }
  ASSERT_TRUE(WaitUntil([&]() {
        if (queue.GetQueueLength() <= 10) {return false;}
        Message msg = queue.GetLatestMessage();
        pcl::PointCloud<Point> p;
        pcl::fromROSMsg<Point>(msg, p);
        return p.points.size() == 3;
      }, 3.0));
  pubs[1]->StopPublishing();
  ASSERT_TRUE(WaitUntil([&]() {
        if (queue.GetQueueLength() <= 10) {return false;}
        Message msg = queue.GetLatestMessage();
        pcl::PointCloud<Point> p;
        pcl::fromROSMsg<Point>(msg, p);
        return p.points.size() == 2;
      }, 3.0));
  queue.ClearCache();
  pubs[1]->StartPublishing<DummySensorTimeForward>(100.0);
  ASSERT_TRUE(WaitUntil([&]() {
        if (queue.GetQueueLength() <= 10) {return false;}
        Message msg = queue.GetLatestMessage();
        pcl::PointCloud<Point> p;
        pcl::fromROSMsg<Point>(msg, p);
        return p.points.size() == 3;
      }, 3.0));
  for (auto pub : pubs) {
    pub->StopPublishing();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  queue.StopCaching();

  Message merged_msg = queue.GetLatestMessage();
  pcl::PointCloud<Point> merged_p;
  pcl::fromROSMsg<Point>(merged_msg, merged_p);

  // verify
  EXPECT_EQ(merged_msg.header.frame_id, "map");
  ASSERT_EQ(merged_p.points.size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(merged_p.points[i].x, expected_points[i].x, kEpsilon);
    EXPECT_NEAR(merged_p.points[i].y, expected_points[i].y, kEpsilon);
    EXPECT_NEAR(merged_p.points[i].z, expected_points[i].z, kEpsilon);
  }
}

// Do not output results unless the topic specified as a trigger is input
TEST_F(NodeTest, CanHushUpPublishingAsLongAsTriggerTopicIsStalled) {
  // setup
  std::vector<CyclicMessagePublisher<Message>::SharedPtr> pubs;
  GeneratePublishers(test_node_->shared_from_this(), pubs);
  auto queue = CacheSubscriber<Message>(test_node_->shared_from_this(), "merged_cloud");
  NodesLaunch();

  // exercise and verify
  ASSERT_TRUE(WaitUntil([&]() {
      for (auto pub : pubs) {
        if (!pub->IsSubscribed()) {return false;}
      }
      return true;
    }, 3.0));
  ASSERT_EQ(queue.GetQueueLength(), 0);
  queue.StartCaching();
  ASSERT_TRUE(WaitUntil([&]() {return queue.IsMessageAvailable();}, 0.5));
  // Publish topics other than the trigger
  pubs[1]->StartPublishing<DummySensorTimeForward>(100.0);
  pubs[2]->StartPublishing<DummySensorTimeForward>(100.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  queue.StopCaching();
  ASSERT_LE(queue.GetQueueLength(), 0);
}

}  // namespace tmc_point_cloud_merger
