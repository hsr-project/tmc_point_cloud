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
#ifndef TMC_POINT_CLOUD_MERGER_PARAM_HPP_
#define TMC_POINT_CLOUD_MERGER_PARAM_HPP_
#include <map>
#include <string>
#include <utility>

namespace tmc_point_cloud_merger {

// TODO(syuuhei_shiro): パラメータ取得関数は共通パッケージに置く
// Retrieve required parameters
template<typename T>
bool GetParam(const rclcpp::Node::SharedPtr& node, const std::string& param_name, T& value) {
  rclcpp::Parameter param;
  if (!node->get_parameter(param_name, param)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("point_cloud_merger"),
      "Parameter '" << param_name << "' is not specified.");
    return false;
  }

  try {
    value = param.get_value<T>();
  } catch (const std::exception& ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("point_cloud_merger"),
      "Get parameter failed: " << ex.what());
    return false;
  }

  return true;
}

template<typename T>
bool GetParam(const std::map<std::string, rclcpp::Parameter>& group, const std::string& param_name, T& value) {
  auto it = group.find(param_name);
  try {
    if (it != group.end()) {
      value = it->second.get_value<T>();
      return true;
    } else {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("point_cloud_merger"),
          "Parameter '" << param_name << "' is not specified.");
      return false;
    }
  } catch (const std::exception& ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("point_cloud_merger"),
      "Get parameter failed: " << ex.what());
    return false;
  }
}

// Retrieve parameter group
inline bool GetGroupParam(const rclcpp::Node::SharedPtr& node, const std::string& group_name,
                          std::map<std::string, rclcpp::Parameter>& group) {
  const bool has_parameters = node->get_parameters(group_name, group);
  if (!has_parameters) {
    RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"),
        "Parameter '%s' group is not specified.", group_name.c_str());
  }
  return has_parameters;
}

inline bool GetGroupParam(const std::map<std::string, rclcpp::Parameter>& parameters, const std::string& group_name,
                          std::map<std::string, rclcpp::Parameter>& group) {
  const std::string prefix = group_name + ".";
  for (auto param : parameters) {
    if (param.first.find(prefix) == 0) {
      std::string param_name = param.first;
      param_name.erase(0, prefix.length());
      group.insert(std::make_pair(param_name, param.second));
    }
  }
  const bool ret = !(group.empty());
  if (!ret) {
    RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"),
        "Parameter '%s' group is not specified.", group_name.c_str());
  }
  return ret;
}
}  // namespace tmc_point_cloud_merger
#endif  // TMC_POINT_CLOUD_MERGER_PARAM_HPP_
