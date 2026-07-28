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
/// @file     named_topic_map-test.cpp
/// @brief    Test for the NamedTopicMap class
/// @author   Fukukazu Kawata
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <tmc_point_cloud_merger/named_topic_map.hpp>

#include "utils.hpp"

namespace tmc_point_cloud_merger {

// A class to verify the artifacts output as a result of user operations
class Observer {
 public:
  Observer() = default;
  ~Observer() = default;

  void Callback(const NamedTopicMap::Subject map) {obs_map_ = map;}
  NamedTopicMap::Subject obs_map() const {return obs_map_;}

 private:
  NamedTopicMap::Subject obs_map_;
};

// Can be initialized successfully
TEST(NamedTopicMapTest, ConstructorOK) {
  // setup
  NamedTopicMap::SharedPtr target;
  Observer obs;

  // exercise
  ASSERT_NO_THROW(target.reset(new NamedTopicMap(
      std::bind(&Observer::Callback, &obs, std::placeholders::_1))));

  // verify
  ASSERT_TRUE(target != nullptr);
}

// Operations with an empty query can be ignored
TEST(NamedTopicMapTest, CanIgnoreLackedQuery) {
  // setup
  Observer obs;
  NamedTopicMap target(
    std::bind(&Observer::Callback, &obs, std::placeholders::_1));

  struct Param {
    std::string topic_name;
    Message::ConstSharedPtr point_cloud;
  };

  std::vector<std::pair<std::string, Param>> test_cases {
    std::make_pair(
      "empty_topic_name",
      Param {"", Message::ConstSharedPtr(new Message(
        CreateTestPointCloud(rclcpp::Time(0), "some_link", 1.0, 1.0, 1.0)))}),
    std::make_pair(
      "empty_point_cloud",
      Param {"some_topic", nullptr})};

  // exercise and verify
  for (auto tc : test_cases) {
    target.UpdateMessage(tc.second.topic_name, tc.second.point_cloud);
    NamedTopicMap::Subject subject = obs.obs_map();
    EXPECT_TRUE(subject.empty()) << "Miss reject has found at test case: " << tc.first;
  }
}

// Artifacts are obtained when valid data is provided
TEST(NamedTopicMapTest, CanNotifyUserOfFrameUpdate) {
  // setup
  Observer obs;
  NamedTopicMap target(
    std::bind(&Observer::Callback, &obs, std::placeholders::_1));
  const std::string some_topic_name("some_topic_name");
  Message::ConstSharedPtr msg(new Message(
      CreateTestPointCloud(rclcpp::Time(0), "some_link", 1.0, 1.0, 1.0)));

  // exercise
  target.UpdateMessage(some_topic_name, msg);
  NamedTopicMap::Subject subject = obs.obs_map();

  // verify
  ASSERT_FALSE(subject.empty());
  ASSERT_NO_THROW(subject.at("some_topic_name"));
  Message::ConstSharedPtr obs_msg = subject.at("some_topic_name");
  ASSERT_TRUE(obs_msg != nullptr);
  pcl::PointCloud<Point> pc;
  pcl::fromROSMsg(*obs_msg, pc);
  ASSERT_FALSE(pc.points.empty());
  EXPECT_EQ(1, static_cast<int>(pc.points.size()));
  EXPECT_NEAR(1.0, pc.points[0].x, 1e-5);
  EXPECT_NEAR(1.0, pc.points[0].y, 1e-5);
  EXPECT_NEAR(1.0, pc.points[0].z, 1e-5);
}

// Artifacts are obtained for delete operations, and it can be confirmed that the data is deleted
TEST(NamedTopicMapTest, CanNotifyUserOfFrameDeletion) {
  // setup
  Observer obs;
  NamedTopicMap target(
    std::bind(&Observer::Callback, &obs, std::placeholders::_1));
  const std::string some_topic_name("some_topic_name");
  Message::ConstSharedPtr msg(new Message(CreateTestPointCloud(
    rclcpp::Time(0), "some_link", 1.0, 1.0, 1.0)));
  target.UpdateMessage(some_topic_name, msg);

  // exercise
  target.EraseMessageByKey("some_topic_name");
  NamedTopicMap::Subject subject = obs.obs_map();

  // verify
  ASSERT_TRUE(subject.empty());
}

// Deletion of non-existent keys or empty string queries can be ignored
TEST(NamedTopicMapTest, CanIgnoreQueryThatNotExists) {
  // setup
  Observer obs;
  NamedTopicMap target(
    std::bind(&Observer::Callback, &obs, std::placeholders::_1));
  const std::string some_topic_name("some_topic_name");
  Message::ConstSharedPtr msg(new Message(CreateTestPointCloud(
    rclcpp::Time(0), "some_link", 1.0, 1.0, 1.0)));
  target.UpdateMessage(some_topic_name, msg);

  // exercise
  target.EraseMessageByKey("difference_topic_name");
  target.EraseMessageByKey("");
  NamedTopicMap::Subject subject = obs.obs_map();

  // verify
  ASSERT_FALSE(subject.empty());
}

}  // end of namespace tmc_point_cloud_merger


int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
