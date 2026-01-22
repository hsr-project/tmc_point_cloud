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
/// @file     utils.hpp
/// @brief    Classes mainly for increasing the reliability of node tests
/// @author   Fukukazu Kawata
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <tmc_point_cloud_merger/type.hpp>

namespace tmc_point_cloud_merger {

using WaitFunctionType = std::function<bool ()>;

bool WaitUntil(WaitFunctionType condition_function, double timeout_sec, double rate_hz = 100.0) {
  // Error checking of arguments
  if (!condition_function) {
    throw std::invalid_argument("Function for waiting is empty.");
  }
  if (timeout_sec < 0.0) {
    throw std::invalid_argument("Timeout must must have fully value");
  }
  if (rate_hz < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("Rate to validate must have fully value");
  }

  const rclcpp::Time end_time = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration::from_seconds(timeout_sec);
  rclcpp::Rate rate(rate_hz);
  while (rclcpp::ok()) {
    if (condition_function()) {
      return true;
    }
    if (rclcpp::Clock(RCL_ROS_TIME).now() >= end_time) {
      break;
    }
    rate.sleep();
  }
  return false;
}

// Load parameters from yaml file
void LoadParameterFromYaml(std::shared_ptr<rclcpp::Node> node,
  const std::string& yaml_directory, const std::string& yaml_name) {
  const std::string yaml_path = yaml_directory + yaml_name;
  // Output yaml_path
  RCLCPP_INFO_STREAM(node->get_logger(), "Load parameter from " << yaml_path);
  // Load yaml and generate ParameterMap
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcl_params_t* yaml_params = rcl_yaml_node_struct_init(allocator);
  rcl_parse_yaml_file(yaml_path.c_str(), yaml_params);
  try {
    rclcpp::ParameterMap yaml_param_map = rclcpp::parameter_map_from(yaml_params);
    rcl_yaml_node_struct_fini(yaml_params);
    // Set ros parameters to node
    const std::string parameter_space = "/" + std::string(node->get_name());
    auto iter = yaml_param_map.find(parameter_space);
    for (auto& param : iter->second) {
      node->set_parameter(param);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(),
        "Failed to load parameter from yaml file: " << e.what());
    rcl_yaml_node_struct_fini(yaml_params);
    return;
  }
}

class IDummySensorTime {
 public:
  using SharedPtr = std::shared_ptr<IDummySensorTime>;
  explicit IDummySensorTime(const rclcpp::Time& base_time) : base_time_(base_time) {}
  virtual ~IDummySensorTime() = default;

  virtual rclcpp::Time GetTime() const = 0;

 protected:
  rclcpp::Time base_time_;
};

class DummySensorTimeForward : public IDummySensorTime {
 public:
  DummySensorTimeForward() : IDummySensorTime(rclcpp::Time(0)) {}
  virtual ~DummySensorTimeForward() = default;

  rclcpp::Time GetTime() const override { return rclcpp::Clock(RCL_ROS_TIME).now(); }
};

class DummySensorTimeBackward : public IDummySensorTime {
 public:
  DummySensorTimeBackward() : IDummySensorTime(rclcpp::Clock(RCL_ROS_TIME).now()) {}
  virtual ~DummySensorTimeBackward() = default;

  rclcpp::Time GetTime() const override {return base_time_ - (rclcpp::Clock(RCL_ROS_TIME).now() - base_time_);}
};

class DummySensorTimeFreezed : public IDummySensorTime {
 public:
  DummySensorTimeFreezed() : IDummySensorTime(rclcpp::Clock(RCL_ROS_TIME).now()) {}
  virtual ~DummySensorTimeFreezed() = default;

  rclcpp::Time GetTime() const override { return base_time_; }
};

template<typename MessageT>
class CyclicMessagePublisher {
 public:
  using SharedPtr = std::shared_ptr<CyclicMessagePublisher>;

  explicit CyclicMessagePublisher(
    const rclcpp::Node::SharedPtr node, const std::string& topic_name, const std::string& frame_id)
  : frame_id_(frame_id), node_(node), msg_() {
    pub_ = node_->create_publisher<MessageT>(topic_name.c_str(), 1);
  }
  virtual ~CyclicMessagePublisher() = default;

  template<class DummySensorTime>
  void StartPublishing(const double rate_hz) {
    sensor_time_.reset(new DummySensorTime());
    if (cyclic_publish_timer_ && cyclic_publish_timer_->is_canceled()) {
      cyclic_publish_timer_->reset();
    }
    cyclic_publish_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz)),
      std::bind(&CyclicMessagePublisher::Publish, this));
  }

  void StopPublishing() {
    cyclic_publish_timer_->cancel();
  }

  bool IsSubscribed() const {
    return pub_->get_subscription_count() != 0;
  }

  // TODO(fukukazu_kawata_zb) Generatorクラスを作って与える形にすれば、値の変化など柔軟に対応できる
  void set_msg(const MessageT& msg) {msg_ = msg;}

 private:
  void Publish() {
    msg_.header.stamp = sensor_time_->GetTime();
    msg_.header.frame_id = frame_id_;
    pub_->publish(msg_);
  }

  const std::string frame_id_;
  rclcpp::Node::SharedPtr node_;
  MessageT msg_;

  std::shared_ptr<rclcpp::Publisher<MessageT>> pub_;
  rclcpp::TimerBase::SharedPtr cyclic_publish_timer_;
  IDummySensorTime::SharedPtr sensor_time_;
};


// message_filters::Cache doesn't reach the itchy spots, so don't use it
template<typename MessageT>
class CacheSubscriber {
 public:
  using SharedPtr = std::shared_ptr<CacheSubscriber>;

  explicit CacheSubscriber(const rclcpp::Node::SharedPtr node, const std::string& topic_name)
  : topic_name_(topic_name), node_(node) {}

  void StartCaching() {
    if (sub_ == nullptr) {
      sub_ = node_->create_subscription<MessageT>(
        topic_name_, 1,
        std::bind(&CacheSubscriber::CacheMessage, this, std::placeholders::_1));
    }
  }

  void StopCaching() {
    if (sub_ != nullptr) {
      sub_.reset();
    }
  }

  void ClearCache() {
    cache_.clear();
  }

  bool IsMessageAvailable() const {return sub_->get_publisher_count() != 0;}
  int GetQueueLength() const {return static_cast<int>(cache_.size());}
  MessageT GetLatestMessage() const {return cache_.back();}

 private:
  void CacheMessage(const MessageT& msg) {
    cache_.push_back(msg);
  }

  const std::string topic_name_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Subscription<MessageT>> sub_;

  std::deque<MessageT> cache_;
};

Message CreateTestPointCloud(
  const rclcpp::Time& stamp, const std::string& frame_id, const double x, const double y, const double z) {
  pcl::PointCloud<Point> p;
  p.points.push_back(Point(x, y, z));
  Message pmsg;
  pcl::toROSMsg<Point>(p, pmsg);
  pmsg.header.frame_id = frame_id;
  pmsg.header.stamp = stamp;
  return pmsg;
}

Message CreateTestPointCloud(
  const rclcpp::Time& stamp, const std::string& frame_id,
  const double x, const double y, const double z,
  const uint8_t r, const uint8_t g, const uint8_t b) {
  pcl::PointCloud<PointRGB> p;
  p.points.push_back(PointRGB(r, g, b));
  p.points[0].x = x;
  p.points[0].y = y;
  p.points[0].z = z;
  Message pmsg;
  pcl::toROSMsg<PointRGB>(p, pmsg);
  pmsg.header.frame_id = frame_id;
  pmsg.header.stamp = stamp;
  return pmsg;
}

Message CreateTestPointCloud(
  const rclcpp::Time& stamp, const std::string& frame_id,
  const double x, const double y, const double z,
  const double i) {
    pcl::PointCloud<PointI> p;
    p.points.push_back(PointI(i));
    p.points[0].x = x;
    p.points[0].y = y;
    p.points[0].z = z;
    Message pmsg;
    pcl::toROSMsg<PointI>(p, pmsg);
    pmsg.header.frame_id = frame_id;
    pmsg.header.stamp = stamp;
    return pmsg;
  }

const std::vector<std::string> GetSubscribedNodeTopics(const rclcpp::Node::SharedPtr node) {
  auto topic_names_and_types = node->get_topic_names_and_types();
  std::vector<std::string> subscribed_topics;
  RCLCPP_INFO(node->get_logger(), "%s node subscribed topics", node->get_name());

  for (const auto& topic : topic_names_and_types) {
    auto topic_name = topic.first;
    auto subscriptions_info = node->get_subscriptions_info_by_topic(topic_name);
    for (const auto& info : subscriptions_info) {
      RCLCPP_INFO(node->get_logger(), "%s, info node: %s", topic_name.c_str(), info.node_name().c_str());
      if (info.node_name() == node->get_name()) {
        subscribed_topics.push_back(topic_name);
      }
    }
  }

  return subscribed_topics;
}

const std::vector<std::string> GetPublishedNodeTopics(const rclcpp::Node::SharedPtr node) {
  auto topic_names_and_types = node->get_topic_names_and_types();
  std::vector<std::string> published_topics;
  RCLCPP_INFO(node->get_logger(), "%s node published topics", node->get_name());

  for (const auto& topic : topic_names_and_types) {
    auto topic_name = topic.first;
    auto publishers_info =  node->get_publishers_info_by_topic(topic_name);
    for (const auto& info : publishers_info) {
      RCLCPP_INFO(node->get_logger(), "%s, info node: %s", topic_name.c_str(), info.node_name().c_str());
      if (info.node_name() == node->get_name()) {
        published_topics.push_back(topic_name);
      }
    }
  }

  return published_topics;
}

std::vector<std::string> Split(const std::string & str, char delimiter) {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    tokens.push_back(token);
  }
  return tokens;
}

// Publish Static Transform
// Input: "x y z yaw pitch roll frame_id child_frame_id"
//   Example: "1.0 1.0 0.0 -0.7853981633974483 0.0 0.0 map s0"
void SendStaticTransform(
  const std::string & input,
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster) {
  auto tokens = Split(input, ' ');
  if (tokens.size() != 8) {
    throw std::runtime_error("Invalid input string");
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = tokens[6];
  transform.child_frame_id = tokens[7];
  transform.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

  transform.transform.translation.x = std::stod(tokens[0]);
  transform.transform.translation.y = std::stod(tokens[1]);
  transform.transform.translation.z = std::stod(tokens[2]);

  tf2::Quaternion q;
  q.setRPY(std::stod(tokens[5]), std::stod(tokens[4]), std::stod(tokens[3]));
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster->sendTransform(transform);
}

}  // namespace tmc_point_cloud_merger
