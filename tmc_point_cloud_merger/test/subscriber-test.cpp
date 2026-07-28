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
/// @file     subscriber-test.cpp
/// @brief    Test for the Subscriber class
/// @author   Fukukazu Kawata
#include <deque>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_point_cloud_merger/subscriber.hpp>

#include "utils.hpp"

namespace {
const double kEpsilon = 1e-6;
}  // anonymous namespace

namespace tmc_point_cloud_merger {

// A class bound to the Subscriber's callback to verify the output
class Observer {
 public:
  Observer() = default;
  ~Observer() = default;

  // They will be bound to test target
  void UpdateCallback(const std::string& topic_name, const Message::ConstSharedPtr points) {
    update_event_queue_.push_back(std::make_pair(topic_name, points));
  }
  void StallCallback(const std::string& topic_name) {
    stall_event_queue_.push_back(topic_name);
  }

  // Getters
  int GetUpdateEventCount() const {return static_cast<int>(update_event_queue_.size());}
  int GetStallEventCount() const {return static_cast<int>(stall_event_queue_.size());}
  std::pair<std::string, Message::ConstSharedPtr> GetLatestUpdateEventValue() const {
    return update_event_queue_.back();
  }
  std::string GetLatestStallEventValue() const {return stall_event_queue_.back();}

 private:
  std::deque<std::pair<std::string, Message::ConstSharedPtr>> update_event_queue_;
  std::deque<std::string> stall_event_queue_;
};

class SubscriberTest : public testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    // Generate a test node
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    test_node_ = std::make_shared<rclcpp::Node>("subscriber_test", options);
    LoadParameterFromYaml(
      test_node_,
      ament_index_cpp::get_package_share_directory("tmc_point_cloud_merger") + "/test/params/",
      "subscriber_test.yaml");
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
    pub_ = test_node_->create_publisher<Message>("points", 1);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
    executor_thread_ = std::make_shared<std::thread>([this]() {
      executor_->spin();
    });
  }

  rclcpp::Node::SharedPtr test_node_;
  Subscriber::SharedPtr subscriber_;
  Observer obs_;
  rclcpp::Publisher<Message>::SharedPtr pub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::shared_ptr<std::thread> executor_thread_;
};

// Can be initialized if valid values are obtained
TEST_F(SubscriberTest, CreateFunctionOK) {
  // setup
  std::map<std::string, rclcpp::Parameter> params;
  ASSERT_TRUE(GetGroupParam(test_node_, "single_param", params));

  auto init = [&]() {
      subscriber_ = Subscriber::Create(
        test_node_->shared_from_this(),
        params,
        std::bind(&Observer::UpdateCallback, &obs_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Observer::StallCallback, &obs_, std::placeholders::_1));
    };

  // exercise
  ASSERT_NO_THROW(init());

  // Retrieve the list of topics the node subscribes to
  auto topics = GetSubscribedNodeTopics(test_node_);

  // verify
  EXPECT_FALSE(subscriber_ == nullptr);
  ASSERT_EQ(2, static_cast<int>(topics.size()));
  EXPECT_STREQ("/camera0/points", topics[0].c_str());
  EXPECT_STREQ("/parameter_events", topics[1].c_str());
}

// Cannot be instantiated if invalid parameters exist on the parameter server
TEST_F(SubscriberTest, CreateFunctionNG) {
  // setup
  std::vector<std::string> invalid_group_names {
    "topic_name_not_found",
    "topic_name_empty",
    "topic_name_invalid_type",
    "stall_monitor_hz_not_found",
    "stall_monitor_hz_small",
    "stall_timeout_sec_not_found",
    "stall_timeout_sec_small"};

  auto init = [&](std::map<std::string, rclcpp::Parameter>& p) {
      subscriber_ = Subscriber::Create(
        test_node_->shared_from_this(),
        p,
        std::bind(&Observer::UpdateCallback, &obs_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Observer::StallCallback, &obs_, std::placeholders::_1));
    };

  std::map<std::string, rclcpp::Parameter> params;
  for (auto group_name : invalid_group_names) {
    ASSERT_TRUE(GetGroupParam(test_node_, group_name, params));
    // exercise and verify
    ASSERT_THROW(init(params), std::invalid_argument) << "No throws by using: " << group_name.c_str();
    params.clear();
  }
}

// Can be initialized using the constructor
TEST_F(SubscriberTest, ConstructorOK) {
  // exercise
  ASSERT_NO_THROW(subscriber_.reset(new Subscriber(
      test_node_->shared_from_this(),
      "some_topic",
      10.0,
      1.0,
      std::bind(&Observer::UpdateCallback, &obs_, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Observer::StallCallback, &obs_, std::placeholders::_1))));

  // Retrieve the list of topics the node subscribes to
  auto topics = GetSubscribedNodeTopics(test_node_);

  // verify
  EXPECT_FALSE(subscriber_ == nullptr);
  ASSERT_EQ(2, static_cast<int>(topics.size()));
  EXPECT_STREQ("/parameter_events", topics[0].c_str());
  EXPECT_STREQ("/some_topic", topics[1].c_str());
}

// Cannot be instantiated if invalid parameters are provided
TEST_F(SubscriberTest, ConstructorNG) {
  struct Param {
    std::string topic_name;
    double stall_monitor_hz;
    double stall_timeout_sec;
  };

  std::vector<std::pair<std::string, Param>> test_cases {
    std::make_pair("topic_name_empty", Param {"", 10.0, 1.0}),
    std::make_pair("stall_monitor_hz_small", Param {"some_topic", 0.0, 1.0}),
    std::make_pair("stall_timeout_sec_small", Param {"some_topic", 10.0, 0.0})};

  // exercise and verify
  for (auto tc : test_cases) {
    EXPECT_THROW(subscriber_.reset(new Subscriber(
        test_node_->shared_from_this(),
        tc.second.topic_name,
        tc.second.stall_monitor_hz,
        tc.second.stall_timeout_sec,
        std::bind(&Observer::UpdateCallback, &obs_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Observer::StallCallback, &obs_, std::placeholders::_1))),
    std::invalid_argument) << "Failed at test case " << tc.first;
  }
}

// A message update event is issued
TEST_F(SubscriberTest, CanNotifyUserOfMessageUpdate) {
  // setup
  NodesLaunch();
  subscriber_.reset(new Subscriber(
      test_node_->shared_from_this(),
      "points",
      10.0,
      0.5,
      std::bind(&Observer::UpdateCallback, &obs_, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Observer::StallCallback, &obs_, std::placeholders::_1)));

  ASSERT_TRUE(WaitUntil([&]() {return pub_->get_subscription_count() != 0;}, 3.0));

  // exercise
  // An event should be triggered for each data reception
  // Assign values to points based on the number of topics (0.0 -> 1.0 -> 2.0)
  for (int topic_id = 0; topic_id < 3; ++topic_id) {
    double v = static_cast<double>(topic_id);
    Message msg = CreateTestPointCloud(rclcpp::Clock(RCL_ROS_TIME).now(), "some_link", v, v, v);
    pub_->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(WaitUntil([&]() {return obs_.GetUpdateEventCount() != topic_id;}, 3.0))
      << "Failed at waiting for topic_id: " << topic_id;
  }
  auto event = obs_.GetLatestUpdateEventValue();

  // verify
  EXPECT_STREQ("points", event.first.c_str());
  pcl::PointCloud<Point> ep;
  pcl::fromROSMsg(*(event.second), ep);
  EXPECT_NEAR(2.0, ep.points[0].x, kEpsilon) << "x: " << ep.points[0].x;
  EXPECT_NEAR(2.0, ep.points[0].y, kEpsilon) << "y: " << ep.points[0].y;
  EXPECT_NEAR(2.0, ep.points[0].z, kEpsilon) << "z: " << ep.points[0].z;
}

// If a message arrives once but then stops, a stall event is issued
TEST_F(SubscriberTest, CanNotifyUserOfMessageStalling) {
  // setup
  NodesLaunch();
  subscriber_.reset(new Subscriber(
      test_node_->shared_from_this(),
      "points",
      10.0,
      0.5,
      std::bind(&Observer::UpdateCallback, &obs_, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Observer::StallCallback, &obs_, std::placeholders::_1)));

  ASSERT_TRUE(WaitUntil([&]() {return pub_->get_subscription_count() != 0;}, 3.0));

  Message msg = CreateTestPointCloud(rclcpp::Clock(RCL_ROS_TIME).now(), "some_link", 1.0, 1.0, 1.0);

  // exercise
  // The monitor is active as soon as the subscriber is instantiated
  // If no messages have arrived, initialization is incomplete and nothing happens
  // Once a message arrives, it starts checking for potential communication stalls
  // Verify that behavior
  ASSERT_FALSE(WaitUntil([&]() {return obs_.GetStallEventCount() != 0;}, 1.0));
  pub_->publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  ASSERT_TRUE(WaitUntil([&]() {return obs_.GetStallEventCount() > 10;}, 3.0));
  std::string topic_name = obs_.GetLatestStallEventValue();

  // verify
  ASSERT_STREQ("points", topic_name.c_str());
}

}  // end of namespace tmc_point_cloud_merger
