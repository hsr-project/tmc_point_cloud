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
/// @file     util.hpp
/// @brief Utility header
/// @author   Kiyohiro Sogen
#ifndef TMC_POINT_CLOUD_UTIL_UTIL_HPP_
#define TMC_POINT_CLOUD_UTIL_UTIL_HPP_

#include <Eigen/Dense>

#include <pcl/common/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tmc_point_cloud_util/macros.hpp"
#include "tmc_point_cloud_util/types.hpp"

namespace tmc_point_cloud_util {

// TODO(park) Vector3dToVector4fは、tmc_eigen_bridgeのros2対応が終わったときに機能を移す
/// @brief Convert Eigen::Vector3d to Eigen::Vector4f
/// @param[in] input Eigen::Vector3d
/// @param[out] output Eigen::Vector4f
void Vector3dToVector4f(const Eigen::Vector3d& input, Eigen::Vector4f& output) {
  output[0] = static_cast<float>(input[0]);
  output[1] = static_cast<float>(input[1]);
  output[2] = static_cast<float>(input[2]);
  output[3] = 0.0;
}

/// @brief Extract point cloud within a region
/// @param[in] point_cloud Input point cloud
/// @param[in] region Region
/// @param[out] dst_point_cloud Output point cloud
template<typename PointT>
void ExtractPointCloudInRegion(const pcl::PointCloud<PointT>& point_cloud,
                               const Region& region,
                               pcl::PointCloud<PointT>& dst_point_cloud) {
  // Get point cloud indices within the range
  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  // pcl represents 3D coordinates using Vector4f.
  Vector3dToVector4f(region.min_point(), min_point);
  Vector3dToVector4f(region.max_point(), max_point);
  // Extract the range within the point cloud
  pcl::CropBox<PointT> crop_box;
  crop_box.setInputCloud(point_cloud.makeShared());
  crop_box.setMin(min_point);
  crop_box.setMax(max_point);
  crop_box.filter(dst_point_cloud);
}

/// @brief Get the region of a box_size area centered at center
/// @param[in] center Center point
/// @param[in] box_size Size of the BOX
/// @param[out] dst_region Region
void GetBoxRegion(const Eigen::Vector3d& center,
                  const Eigen::Vector3d& box_size,
                  Region& dst_region);


}  // namespace tmc_point_cloud_util
#endif  // TMC_POINT_CLOUD_UTIL_UTIL_HPP_
