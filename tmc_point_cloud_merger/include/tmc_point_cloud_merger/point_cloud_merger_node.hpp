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
/// @file     point_cloud_merger_node.hpp
/// @brief    Node for point cloud synthesizer
/// @author   Fukukazu Kawata
#ifndef TMC_POINT_CLOUD_MERGER_POINT_CLOUD_MERGER_NODE_HPP_
#define TMC_POINT_CLOUD_MERGER_POINT_CLOUD_MERGER_NODE_HPP_
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "merge_trigger.hpp"
#include "merger.hpp"
#include "named_topic_map.hpp"
#include "param.hpp"
#include "subscriber.hpp"

namespace tmc_point_cloud_merger {

// Connect callbacks of each class and expose functionality as a ROS node for the merger
class PointCloudMergerNode : public rclcpp::Node {
 public:
  explicit PointCloudMergerNode(const rclcpp::NodeOptions& options)
      : Node("point_cloud_merger", options) {}

  /// Initialization
  void Init() {
    // Initialization of publisher that have a duty for merginalization of clouds
    std::map<std::string, rclcpp::Parameter> output_params;
    if (!GetGroupParam(shared_from_this(), "output", output_params)) {
      throw std::invalid_argument("Failed to get parameters for output");
    }
    merger_pub_ = MergedCloudPublisher::Create(shared_from_this(), output_params);

    // Initialization of trigger of merger
    std::map<std::string, rclcpp::Parameter> trigger_params;
    if (!GetGroupParam(shared_from_this(), "trigger", trigger_params)) {
      throw std::invalid_argument("Failed to get parameters for trigger");
    }
    merge_trigger_ = MergeTrigger::Create(
        trigger_params,
        std::bind(&MergedCloudPublisher::MergeAndPublish, merger_pub_.get(), std::placeholders::_1));

    named_topic_map_.reset(new NamedTopicMap(
        std::bind(&MergeTrigger::UpdateMessageFrame, merge_trigger_.get(), std::placeholders::_1)));

    // Initialization of inputs
    std::map<std::string, rclcpp::Parameter> inputs_params;
    if (!GetGroupParam(shared_from_this(), "inputs", inputs_params)) {
      throw std::invalid_argument("Failed to get parameters for inputs");
    }
    std::vector<std::string> registered_list;
    for (auto [key, param] : inputs_params) {
      const size_t substr_index = key.find(".");
      if (substr_index == std::string::npos) {
        throw std::invalid_argument("Failed to get parameters for input");
      }
      const std::string input_name = key.substr(0, substr_index);
      if (std::find(registered_list.begin(), registered_list.end(), input_name) != registered_list.end()) {
        // Registered
        continue;
      }
      std::map<std::string, rclcpp::Parameter> input_params;
      GetGroupParam(inputs_params, input_name, input_params);
      sub_.push_back(Subscriber::Create(shared_from_this(), input_params,
          std::bind(&NamedTopicMap::UpdateMessage, named_topic_map_.get(),
              std::placeholders::_1, std::placeholders::_2),
          std::bind(&NamedTopicMap::EraseMessageByKey, named_topic_map_.get(), std::placeholders::_1)));
      registered_list.push_back(input_name);
    }
    RCLCPP_INFO(this->get_logger(), "%s has initialized", this->get_name());
  }

 private:
  std::vector<Subscriber::SharedPtr> sub_;
  NamedTopicMap::SharedPtr named_topic_map_;
  MergeTrigger::SharedPtr merge_trigger_;
  MergedCloudPublisher::SharedPtr merger_pub_;
};
}  // end of namespace tmc_point_cloud_merger
#endif  // TMC_POINT_CLOUD_MERGER_POINT_CLOUD_MERGER_NODE_HPP_
