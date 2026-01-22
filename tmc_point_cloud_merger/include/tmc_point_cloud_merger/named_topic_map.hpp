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
/// @file     named_topic_map.hpp
/// @brief    Manage named PointClouds using a map and provide necessary interfaces
/// @author   Fukukazu Kawata
#ifndef TMC_POINT_CLOUD_MERGER_NAMED_TOPIC_MAP_HPP_
#define TMC_POINT_CLOUD_MERGER_NAMED_TOPIC_MAP_HPP_
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "type.hpp"

namespace tmc_point_cloud_merger {


// Manage messages with data and names
// Can add and remove
// Notify the upper layer when changes occur
class NamedTopicMap {
 public:
  using SharedPtr = std::shared_ptr<NamedTopicMap>;
  using Subject = std::map<std::string, Message::ConstSharedPtr>;
  using CallbackType = std::function<void(const Subject)>;

  explicit NamedTopicMap(CallbackType callback) : callback_(callback) {}

  void UpdateMessage(const std::string& topic_name, const Message::ConstSharedPtr msg) {
    if (topic_name.empty() || msg == nullptr) return;
    // If a non-existent key is specified, it will be added
    msg_by_name_[topic_name] = msg;
    callback_(msg_by_name_);
  }

  void EraseMessageByKey(const std::string& topic_name) {
    // If a non-existent key is specified, it is meaningless and will be ignored
    if (topic_name.empty() || msg_by_name_.count(topic_name) == 0) {
      return;
    }
    msg_by_name_.erase(topic_name);
    callback_(msg_by_name_);
  }

 private:
  CallbackType callback_;
  Subject msg_by_name_;
};

}  // end of namespace tmc_point_cloud_merger

#endif  // TMC_POINT_CLOUD_MERGER_NAMED_TOPIC_MAP_HPP_
