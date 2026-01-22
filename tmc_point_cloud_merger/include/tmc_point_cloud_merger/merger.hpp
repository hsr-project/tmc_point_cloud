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
/// @file     merger.hpp
/// @brief    Convert multiple obtained point clouds to the reference coordinate system, then merge and publish them
/// @author   Fukukazu Kawata
#ifndef TMC_POINT_CLOUD_MERGER_MERGER_HPP_
#define TMC_POINT_CLOUD_MERGER_MERGER_HPP_
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>

#include "param.hpp"
#include "type.hpp"


namespace tmc_point_cloud_merger {

// Merge the given data and publish it
class MergedCloudPublisher {
 public:
  using SharedPtr = std::shared_ptr<MergedCloudPublisher>;
  using Subject = std::map<std::string, Message::ConstSharedPtr>;

  static SharedPtr Create(const rclcpp::Node::SharedPtr node,
    std::map<std::string, rclcpp::Parameter>& params) {
    std::string topic_name;
    std::string frame_id;
    if (!GetParam(params, "topic_name", topic_name) ||
        !GetParam(params, "frame_id", frame_id)) {
      throw std::invalid_argument("Failed to get parameters for output");
    }
    return SharedPtr(new MergedCloudPublisher(node, topic_name, frame_id));
  }

  MergedCloudPublisher(
    const rclcpp::Node::SharedPtr node,
    const std::string& topic_name, const std::string& frame_id)
  : frame_id_(frame_id) {
    // Publisher does not provide any information if it's an empty string
    if (topic_name.empty()) {
      throw std::invalid_argument("Specified topic name is empty");
    }
    if (frame_id_.empty()) {
      throw std::invalid_argument("Specified frame id is empty");
    }

    buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    pub_ = node->create_publisher<Message>(topic_name, 1);
  }

  // Transform coordinates based on the frame_id specified in the member and merge/publish
  void MergeAndPublish(const Subject& frame) {
    // No work if it's empty
    if (frame.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"), "Input frame has no data");
      return;
    }

    // Obtain and apply rigid body transformation to each sensor's relative reference frame_id
    std::vector<Message::SharedPtr> transformed_clouds;
    // Could be written more coolly with C++17
    // TODO(fukukazu_kawata_zb) 並列化も視野に。
    for (auto it = frame.begin(); it != frame.end(); ++it) {
      if (it->second == nullptr) {
        RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"), "Empty pointer has detected: %s", it->first.c_str());
        continue;
      }
      if (it->second->data.empty()) {
        // Warning abolished as it's quite possible for the number of point clouds to become zero after filtering
        // RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"), "No points in: %s", it->first.c_str());
        continue;
      }
      // If the specified frame_id is that of the sensor, proceed to the next step as is
      if ((it->second)->header.frame_id == frame_id_) {
        Message::SharedPtr c(new Message(*(it->second)));
        transformed_clouds.push_back(c);
        continue;
      }
      // transform from sensor to target
      // https://qiita.com/yakato_jun/items/091f36308b0662110537
      Eigen::Matrix4f t;
      try {
        const geometry_msgs::msg::TransformStamped ts = buffer_->lookupTransform(
          frame_id_, (it->second)->header.frame_id, (it->second)->header.stamp,
          rclcpp::Duration::from_seconds(1.0));
        t = tf2::transformToEigen(ts).matrix().cast<float>();
      } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"), "Failed to lookup transform from sensor to %s: %s",
          frame_id_.c_str(), e.what());
        continue;
      }
      Message::SharedPtr transformed_cloud(new Message());
      pcl_ros::transformPointCloud(t, *(it->second), *transformed_cloud);
      transformed_clouds.push_back(transformed_cloud);
    }
    // Merge
    if (transformed_clouds.empty()) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("point_cloud_merger"), clock, 1000,
        "No clouds to merge");
      return;
    }
    Message::SharedPtr merged_cloud(new Message());
    for (auto transformed_cloud : transformed_clouds) {
      Message::SharedPtr pc_xyz = ExtractXYZData(transformed_cloud);
      MergeTwoCloudsIntoOne(pc_xyz, merged_cloud);
    }

    if (merged_cloud->data.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("point_cloud_merger"), "No clouds are merged");
      return;
    }

    merged_cloud->header.frame_id = frame_id_;
    pub_->publish(*merged_cloud);
  }

 private:
  // Wrapper for pcl::concatenatePointCloud
  // If it's designed to always require three objects for synthesis,
  // You need to implement a recursive function to ultimately synthesize into a single data
  // Make it two inputs for a simple for loop, with one returning the merged data
  void MergeTwoCloudsIntoOne(const Message::SharedPtr in, Message::SharedPtr& inout) {
    Message::SharedPtr out(new Message());
    pcl::concatenatePointCloud(*inout, *in, *out);
    inout = out;
    inout->header.stamp = rclcpp::Time(in->header.stamp) > rclcpp::Time(inout->header.stamp) ?
      in->header.stamp : inout->header.stamp;
  }

  // Extract only XYZ data
  Message::SharedPtr ExtractXYZData(const Message::SharedPtr& in) {
    // Immediately return if there are no elements other than x, y, z and no processing is needed
    std::vector<sensor_msgs::msg::PointField> non_xyz_fields;
    std::copy_if(
      in->fields.begin(), in->fields.end(),
      std::back_inserter(non_xyz_fields),
      [](const sensor_msgs::msg::PointField& f) {return f.name != "x" && f.name != "y" && f.name != "z";});
    if (non_xyz_fields.empty()) {
      return in;
    }

    // Filtering can be done by transforming while ignoring elements other than x, y, z
    pcl::PointCloud<Point> pc;
    pcl::fromROSMsg<Point>(*in, pc);
    Message::SharedPtr out(new Message());
    pcl::toROSMsg<Point>(pc, *out);
    // In pcl transformation, the timestamp precision decreases, so reinsert it
    out->header = in->header;

    return out;
  }

  const std::string frame_id_;
  rclcpp::Publisher<Message>::SharedPtr pub_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};
}  // end of namespace tmc_point_cloud_merger
#endif  // TMC_POINT_CLOUD_MERGER_MERGER_HPP_
