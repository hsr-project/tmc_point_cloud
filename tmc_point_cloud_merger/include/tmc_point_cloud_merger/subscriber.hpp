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
/// @file     subscriber.hpp
/// @brief    Subscriber with communication status monitoring functionality
/// @author   Fukukazu Kawata
#ifndef TMC_POINT_CLOUD_MERGER_SUBSCRIBER_HPP_
#define TMC_POINT_CLOUD_MERGER_SUBSCRIBER_HPP_
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "param.hpp"
#include "type.hpp"

namespace tmc_point_cloud_merger {
using std::chrono::milliseconds;
using std::placeholders::_1;

// Subscribe to messages and relay them to the upper layer
// Notify the upper layer even when message updates are interrupted
class Subscriber {
 public:
  using SharedPtr = std::shared_ptr<Subscriber>;
  // Receive event function type
  using UpdateCallbackType = std::function<void(const std::string&, const Message::ConstSharedPtr)>;
  // Function type for when messages stop arriving
  using StallCallbackType = std::function<void(const std::string&)>;

  static SharedPtr Create(const rclcpp::Node::SharedPtr node,
      std::map<std::string, rclcpp::Parameter>& params,
      UpdateCallbackType update_callback,
      StallCallbackType stall_callback) {
    std::string topic_name;
    double stall_monitor_hz;
    double stall_timeout_sec;
    if (!GetParam(params, "topic_name", topic_name) ||
        !GetParam(params, "stall_monitor_hz", stall_monitor_hz) ||
        !GetParam(params, "stall_timeout_sec", stall_timeout_sec)) {
      throw std::invalid_argument("Failed to get parameters for input");
    }
    return SharedPtr(
        new Subscriber(node,
            topic_name, stall_monitor_hz, stall_timeout_sec,
            update_callback, stall_callback));
  }

  // Subscriber activation, etc.
  Subscriber(const rclcpp::Node::SharedPtr node,
      const std::string& topic_name,
      const double stall_monitor_hz,
      const double stall_timeout,
      UpdateCallbackType update_callback,
      StallCallbackType stall_callback)
      : node_(node),
        topic_name_(topic_name),
        stall_timeout_(rclcpp::Duration::from_seconds(stall_timeout)),
        update_callback_(update_callback),
        stall_callback_(stall_callback),
        latest_stamp_(0, 0, RCL_ROS_TIME)  {
    if (topic_name_.empty()) {
      throw std::invalid_argument("Topic name is empty. Please specify the name to subscribe");
    }
    if (stall_monitor_hz < 1.0e-5) {
      throw std::invalid_argument("Specfied rate is very small");
    }
    // Extremely small timeouts are not allowed as they cause frequent stall events
    if (stall_timeout_ < rclcpp::Duration::from_seconds(1.0e-5)) {
      throw std::invalid_argument("Specfied timeout is very small");
    }

    sub_ = node->create_subscription<Message>(
      topic_name_, rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Subscriber::UpdateMessage, this, _1));

    monitor_event_ = node->create_wall_timer(milliseconds(static_cast<int32_t>(1000 / stall_monitor_hz)),
        std::bind(&Subscriber::UpdateStamp, this));
  }

  virtual ~Subscriber() = default;

 private:
  void UpdateMessage(const Message::SharedPtr msg) {
    latest_stamp_ = node_->now();
    update_callback_(topic_name_, msg);
  }

  void UpdateStamp() {
    if (latest_stamp_ == rclcpp::Time(0, 0, RCL_ROS_TIME)) {
      auto clock = node_->get_clock();
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("point_cloud_merger"), *clock, 5000,
          "Waiting for first message: %s", topic_name_.c_str());
      return;
    }
    if (node_->now() - latest_stamp_ > stall_timeout_) {
      auto clock = node_->get_clock();
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("point_cloud_merger"), *clock, 1000,
          "Stalling has detected: %s", topic_name_.c_str());
      stall_callback_(topic_name_);
    }
  }

  rclcpp::Node::SharedPtr node_;
  const std::string topic_name_;
  const rclcpp::Duration stall_timeout_;

  UpdateCallbackType update_callback_;
  StallCallbackType stall_callback_;

  rclcpp::Subscription<Message>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr monitor_event_;

  rclcpp::Time latest_stamp_;
};

}  // end of namespace tmc_point_cloud_merger

#endif  // TMC_POINT_CLOUD_MERGER_SUBSCRIBER_HPP_
