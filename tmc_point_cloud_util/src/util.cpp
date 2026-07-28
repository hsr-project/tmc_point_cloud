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
/// @file     util.cpp
/// @brief    Utility
/// @author   Kiyohiro Sogen

#include <tmc_point_cloud_util/util.hpp>

#include <limits>

namespace tmc_point_cloud_util {

/// Get the region of a box with size box_size centered at center
void GetBoxRegion(const Eigen::Vector3d& center,
                  const Eigen::Vector3d& box_size,
                  Region& dst_region) {
  if ((box_size[0] <= std::numeric_limits<double>::min()) ||
      (box_size[1] <= std::numeric_limits<double>::min()) ||
      (box_size[2] <= std::numeric_limits<double>::min())) {
    PCU_THROW_RUNTIME_ERROR("box_size must be positive.");
  }
  Eigen::Vector3d min_point;
  min_point = center - box_size / 2.0;
  dst_region.set_min_point(min_point);

  Eigen::Vector3d max_point;
  max_point = center + box_size / 2.0;
  dst_region.set_max_point(max_point);
}

}  // namespace tmc_point_cloud_util
