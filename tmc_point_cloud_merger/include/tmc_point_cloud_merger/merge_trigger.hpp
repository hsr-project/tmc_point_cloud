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
/// @file     merge_trigger.hpp
/// @brief    A class that determines whether to merge based on changes between the latest data and past data and notifies accordingly
/// @author   Fukukazu Kawata
#ifndef TMC_POINT_CLOUD_MERGER_MERGE_TRIGGER_HPP_
#define TMC_POINT_CLOUD_MERGER_MERGE_TRIGGER_HPP_
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "param.hpp"
#include "type.hpp"

namespace tmc_point_cloud_merger {

// Notifies upstream when the specified topic name is updated
// Also notifies when there is a change in the number of types of data being handled
class MergeTrigger {
 public:
  using SharedPtr = std::shared_ptr<MergeTrigger>;
  using Subject = std::map<std::string, Message::ConstSharedPtr>;
  using CallbackType = std::function<void(const Subject)>;

  static SharedPtr Create(
      std::map<std::string, rclcpp::Parameter>& params,
      CallbackType callback) {
    std::vector<std::string> topic_names;
    for (auto [key, param] : params) {
      const size_t substr_index = key.find(".");
      if (substr_index == std::string::npos) {
        throw std::invalid_argument("Failed to get parameters for trigger");
      }
      const std::string trigger_name = key.substr(0, substr_index);
      if (std::find(topic_names.begin(), topic_names.end(), trigger_name) != topic_names.end()) {
        // Registered
        continue;
      }
      std::map<std::string, rclcpp::Parameter> trigger_params;
      if (!GetGroupParam(params, trigger_name, trigger_params)) {
        throw std::invalid_argument("Failed to get parameters for trigger");
      }
      std::string topic_name;
      if (!GetParam(trigger_params, "topic_name", topic_name)) {
        throw std::invalid_argument("Failed to get parameters for trigger");
      }
      topic_names.push_back(topic_name);
    }
    return SharedPtr(new MergeTrigger(topic_names, callback));
  }


  explicit MergeTrigger(
      const std::vector<std::string>& topic_names,
      CallbackType callback)
      : topic_names_(topic_names), callback_(callback) {
    if (topic_names_.empty()) {
      throw std::invalid_argument("Topic name(s) for notification to the upper layer is empty");
    }
    if (std::set<std::string>(topic_names_.begin(), topic_names_.end()).size() != topic_names_.size()) {
      throw std::invalid_argument("Contains the same targets");
    }
    for (auto s : topic_names_) {
      if (s.empty()) throw std::invalid_argument("A element of topic_names is empty");
    }
  }

  void UpdateMessageFrame(const Subject& frame) {
    // If not initialized and the input is empty, there is no task
    if (frame.empty() && latest_frame_.empty()) {
      return;
    }

    // Notifies when the data for the specified topic name is updated
    // Relies on timestamps
    for (auto topic_name : topic_names_) {
      try {
        // Always notifies when a trigger is received for the first time
        // Subsequently, determines whether to notify based on whether the timestamp has been updated
        if (latest_frame_.find(topic_name) == latest_frame_.end() && frame.find(topic_name) != frame.end()) {
          latest_frame_ = frame;
          callback_(latest_frame_);
          return;
        }
        // Existence check runs with .at
        // Throws an exception if not found
        if (rclcpp::Time(frame.at(topic_name)->header.stamp) >
            rclcpp::Time(latest_frame_.at(topic_name)->header.stamp)) {
          latest_frame_ = frame;
          callback_(latest_frame_);
          return;
        }
      } catch (const std::out_of_range& e) {
        auto clock = rclcpp::Clock(RCL_ROS_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("point_cloud_merger"), clock, 1000,
            "No specified topic data in buffer: %s", topic_name.c_str());
        continue;
      }
    }

    latest_frame_ = frame;
  }

 private:
  const std::vector<std::string> topic_names_;
  CallbackType callback_;
  Subject latest_frame_;
};

}  // end of namespace tmc_point_cloud_merger

#endif  // TMC_POINT_CLOUD_MERGER_MERGE_TRIGGER_HPP_
