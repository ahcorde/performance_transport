// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "performance_transport/point_cloud_transport/PublisherPointCloudTransport.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

namespace performance_transport
{
PublisherPointCloudTransport::PublisherPointCloudTransport(
  const rclcpp::NodeOptions & _options,
  const std::string & _filename)
: Node("publisher_point_cloud_transport", _options),
  filename_(_filename)
{
  timer_ = this->create_wall_timer(
    33ms,
    std::bind(&PublisherPointCloudTransport::PublishMessage, this));
}

void PublisherPointCloudTransport::PublishMessage()
{
  // std::lock_guard<std::mutex> lock(mutex_);
  this->cloud_msg_.header.stamp = this->now();
  this->cloud_msg_.header.frame_id = "camera_link";
  this->pub_.publish(this->cloud_msg_);
}

void PublisherPointCloudTransport::Initialize()
{
  this->pc_ =
    std::make_shared<point_cloud_transport::PointCloudTransport>(this->shared_from_this());
  this->pub_ = this->pc_->advertise("pct/point_cloud", 10);
  if (this->filename_ == "" || pcl::io::loadPCDFile(this->filename_, this->cloud_msg_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "failed to open PCD file");
    throw std::runtime_error{"could not open pcd file"};
  }
}

}  // namespace performance_transport
