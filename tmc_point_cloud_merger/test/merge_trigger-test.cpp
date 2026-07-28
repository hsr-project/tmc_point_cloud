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
/// @file     merge_trigger-test.cpp
/// @brief    Test for the MergeTrigger class
/// @author   Fukukazu Kawata

#include <deque>
#include <functional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmc_point_cloud_merger/merge_trigger.hpp>

#include "utils.hpp"

namespace tmc_point_cloud_merger {

// Observer to obtain the product
class Observer {
 public:
  Observer() = default;
  ~Observer() = default;

  void Callback(const MergeTrigger::Subject s) {subjects_.push_back(s);}
  MergeTrigger::Subject GetLatestSubject() const {return subjects_.back();}
  int GetCallbackCount() const {return subjects_.size();}

 private:
  std::deque<MergeTrigger::Subject> subjects_;
};

// Class for testing the generation function
class CreateFunctionTest : public testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    test_node_ = std::make_shared<rclcpp::Node>("merger_trigger_test", options);

    // Load parameters for testing
    LoadParameterFromYaml(
      test_node_,
      ament_index_cpp::get_package_share_directory("tmc_point_cloud_merger") + "/test/params/",
      "merger_trigger_test.yaml");
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr test_node_;
  MergeTrigger::SharedPtr merge_trigger_;
  Observer obs_;
};

// Whether single or multiple, it should succeed if the parameters are appropriate
TEST_F(CreateFunctionTest, SucceedToCreate) {
  // setup
  std::vector<std::string> valid_group_names {
    "valid_param_single",
    "valid_param_multiple"
  };

  auto init = [&](std::map<std::string, rclcpp::Parameter>& p) {
      merge_trigger_ = MergeTrigger::Create(
        p,
        std::bind(&Observer::Callback, &obs_, std::placeholders::_1));
    };

  std::map<std::string, rclcpp::Parameter> params;
  for (auto group_name : valid_group_names) {
    ASSERT_TRUE(GetGroupParam(test_node_, group_name, params));
    EXPECT_NO_THROW(init(params));
    // exercise and verify
    EXPECT_TRUE(merge_trigger_ != nullptr);
    params.clear();
  }
}


// Invalid settings such as duplicates, emptiness, or missing keys should not be allowed
TEST_F(CreateFunctionTest, FailedToCreate) {
  // setup
  std::vector<std::string> invalid_group_names {
    "has_the_same_targets",
    "invalid_key",
    "empty"
  };

  auto init = [&](std::map<std::string, rclcpp::Parameter>& p) {
      merge_trigger_ = MergeTrigger::Create(
        p,
        std::bind(&Observer::Callback, &obs_, std::placeholders::_1));
    };

  // exercise and verify
  std::map<std::string, rclcpp::Parameter> params;
  for (auto group_name : invalid_group_names) {
    ASSERT_TRUE(GetGroupParam(test_node_, group_name, params));
    EXPECT_THROW(init(params), std::invalid_argument);
    params.clear();
  }
}

// Constructor test parameters
struct ConstructorTestParam {
  std::vector<std::string> topic_names;
};

// Normal case
class ConstructorTestOK : public testing::TestWithParam<ConstructorTestParam> {};

INSTANTIATE_TEST_CASE_P(
  ConstructorOK,
  ConstructorTestOK,
  testing::Values(
    ConstructorTestParam{{"some_topic"}},
    ConstructorTestParam{{"some_topic_0", "some_topic_1"}}));


TEST_P(ConstructorTestOK, ConstructorOK) {
  // setup
  MergeTrigger::SharedPtr target;
  Observer obs;

  // exercise
  ASSERT_NO_THROW(target.reset(new MergeTrigger(
      GetParam().topic_names,
      std::bind(&Observer::Callback, &obs, std::placeholders::_1))));

  // verify
  ASSERT_TRUE(target != nullptr);
}

// Abnormal case
class ConstructorTestNG : public testing::TestWithParam<ConstructorTestParam> {};

INSTANTIATE_TEST_CASE_P(
  ConstructorNG,
  ConstructorTestNG,
  testing::Values(
    ConstructorTestParam{{""}},
    ConstructorTestParam{{"same_topic", "same_topic"}},
    ConstructorTestParam{{}}));

TEST_P(ConstructorTestNG, ConstructorNG) {
  // setup
  MergeTrigger::SharedPtr target;
  Observer obs;

  // exercise
  EXPECT_THROW(target.reset(new MergeTrigger(
      GetParam().topic_names,
      std::bind(&Observer::Callback, &obs, std::placeholders::_1))),
    std::invalid_argument);

  // verify
  ASSERT_TRUE(target == nullptr);
}

// Behavior test fixture
class BehaviorTest : public testing::Test {
 public:
  void SetUp() override {
    ASSERT_NO_THROW(merge_trigger_.reset(new MergeTrigger(
        {"/camera0/points", "/camera1/points"},
        std::bind(&Observer::Callback, &obs_, std::placeholders::_1))));
    ASSERT_TRUE(merge_trigger_ != nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

 protected:
  MergeTrigger::SharedPtr merge_trigger_;
  Observer obs_;
};

// Should ignore even if an empty map is provided
TEST_F(BehaviorTest, CanIgnoreEmptyFrame) {
  // setup
  MergeTrigger::Subject subject;

  // exercise
  merge_trigger_->UpdateMessageFrame(subject);

  // verify
  ASSERT_EQ(0, obs_.GetCallbackCount());
}

// Confirm response containing the specified topic
TEST_F(BehaviorTest, ExpectedIgnoreAndNotification) {
  // setup
  MergeTrigger::Subject some_frame {
    {"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Time(0), "any_link", 1.0, 1.0, 1.0)))}};

  MergeTrigger::Subject target_frame {
    {"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Time(0), "any_link", 1.0, 1.0, 1.0)))},
    {"/camera0/points", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Time(0), "any_link", 1.0, 1.0, 1.0)))}};

  MergeTrigger::Subject target_frame_updated {
    {"any_topic_name", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Time(0), "any_link", 1.0, 1.0, 1.0)))},
    {"/camera0/points", Message::ConstSharedPtr(new Message(CreateTestPointCloud(
      rclcpp::Time(1), "any_link", 1.0, 1.0, 1.0)))}};

  // exercise
  // Not the target, so no notification
  merge_trigger_->UpdateMessageFrame(some_frame);
  const int first_event_count = obs_.GetCallbackCount();
  RCLCPP_INFO(rclcpp::get_logger(""), "%d", first_event_count);

  // Is the target, so notified (count += 1)
  merge_trigger_->UpdateMessageFrame(target_frame);
  const int second_event_count = obs_.GetCallbackCount();
  RCLCPP_INFO(rclcpp::get_logger(""), "%d", second_event_count);

  // Contains the target, but no notification as the stamp hasn't changed
  merge_trigger_->UpdateMessageFrame(target_frame);
  const int third_event_count = obs_.GetCallbackCount();
  RCLCPP_INFO(rclcpp::get_logger(""), "%d", third_event_count);

  // Is the target, and the stamp is updated, so notified
  merge_trigger_->UpdateMessageFrame(target_frame_updated);
  const int final_event_count = obs_.GetCallbackCount();
  RCLCPP_INFO(rclcpp::get_logger(""), "%d", final_event_count);

  // verify
  EXPECT_EQ(0, first_event_count);
  EXPECT_EQ(1, second_event_count);
  EXPECT_EQ(1, third_event_count);
  EXPECT_EQ(2, final_event_count);
}

}  // end of namespace tmc_point_cloud_merger
