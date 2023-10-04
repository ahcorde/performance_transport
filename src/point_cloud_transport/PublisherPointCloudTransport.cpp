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

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace performance_transport
{
PublisherPointCloudTransport::PublisherPointCloudTransport(
  const rclcpp::NodeOptions & _options)
: Node("publisher_point_cloud_transport", _options)
{
  this->timer_ = this->create_wall_timer(
    33ms,
    std::bind(&PublisherPointCloudTransport::PublishMessage, this));
}

PublisherPointCloudTransport::~PublisherPointCloudTransport()
{
  this->Destroy();
}

void PublisherPointCloudTransport::Destroy()
{
  this->pub_.shutdown();
  this->pc_.reset();
  this->timer_->cancel();
}

void PublisherPointCloudTransport::SetFilename(
  const std::string & _filename)
{
  this->filename_ = _filename;
}

void PublisherPointCloudTransport::SetCompressType(
  const std::string & _compress_type)
{
  this->compress_type_ = _compress_type;
}

void PublisherPointCloudTransport::SetCompress(int _value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->compress_ = _value;
}

int PublisherPointCloudTransport::GetSize()
{
  return cloud_msg_.height * cloud_msg_.width * cloud_msg_.fields.size();
}

int PublisherPointCloudTransport::GetNumberOfImagesPublished()
{
  std::lock_guard<std::mutex> lock(mutex_);
  int result = this->count_;
  this->count_ = 0;
  return result;
}

void PublisherPointCloudTransport::PublishMessage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->cloud_msg_.header.stamp = this->now();
  this->cloud_msg_.header.frame_id = "camera_link";
  this->pub_.publish(this->cloud_msg_);
  if (this->count_ == 0) {
    this->SetCompressParameter();
  }
  this->count_++;
}

void PublisherPointCloudTransport::SetTransportHint(
  const std::string & _transport_hint)
{
  this->transport_hint_ = _transport_hint;
}

void PublisherPointCloudTransport::SetCompressParameter()
{
  if (this->transport_hint_ == "draco") {
    this->set_parameter(rclcpp::Parameter("pct.point_cloud.draco.encode_speed", this->compress_));
    this->set_parameter(rclcpp::Parameter("pct.point_cloud.draco.decode_speed", this->compress_));
  } else if (this->transport_hint_ == "zlib") {
    this->set_parameter(rclcpp::Parameter("pct.point_cloud.zlib.encode_level", this->compress_));
  } else if (this->transport_hint_ == "zstd") {
    this->set_parameter(
      rclcpp::Parameter(
        "pct.point_cloud.zstd.zstd_encode_level", this->compress_));
  }
}

void PublisherPointCloudTransport::Initialize()
{
  this->pc_ =
    std::make_shared<point_cloud_transport::PointCloudTransport>(this->shared_from_this());
  this->pub_ = this->pc_->advertise("pct/point_cloud", rmw_qos_profile_sensor_data);
  if (this->filename_ == "" || pcl::io::loadPCDFile(this->filename_, this->cloud_msg_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "failed to open PCD file");
    throw std::runtime_error{"could not open PCD file"};
  }
}

}  // namespace performance_transport
