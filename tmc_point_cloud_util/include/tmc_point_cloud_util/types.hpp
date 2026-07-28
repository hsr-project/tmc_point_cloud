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
/// @file     types.hpp
/// @brief    Common type definition header
/// @author   Kiyohiro Sogen
#ifndef TMC_POINT_CLOUD_UTIL_TYPES_HPP_
#define TMC_POINT_CLOUD_UTIL_TYPES_HPP_

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace tmc_point_cloud_util {

/// Point cloud index
using Indices = std::vector<int32_t>;
using IndicesPtr = std::shared_ptr<Indices>;
using IndicesConstPtr = std::shared_ptr<const Indices>;

/// Model parameters
typedef Eigen::VectorXd ModelCoefficients;
/// PointCloud2
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

/// @class Region
/// @brief Region class
class Region {
 public:
  /// @brief Constructor
  Region() {}
  /// @brief Constructor
  /// @param[in] min_point Minimum value
  /// @param[in] max_point Maximum value
  Region(const Eigen::Vector3d& min_point,
         const Eigen::Vector3d& max_point)
      : min_point_(min_point),
        max_point_(max_point) {}

  /// Accessor
  Eigen::Vector3d min_point() const {
    return min_point_;
  }
  Eigen::Vector3d max_point() const {
    return max_point_;
  }
  /// Mutator
  void set_min_point(const Eigen::Vector3d& min_point) {
    min_point_ = min_point;
  }
  void set_max_point(const Eigen::Vector3d& max_point) {
    max_point_ = max_point;
  }

 private:
  /// Minimum value
  Eigen::Vector3d min_point_;
  /// Maximum value
  Eigen::Vector3d max_point_;
};
}  // namespace tmc_point_cloud_util
#endif  // TMC_POINT_CLOUD_UTIL_TYPES_HPP_
