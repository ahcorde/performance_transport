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

#include "performance_transport/image_transport/PublisherImageTransport.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>

#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

namespace performance_transport
{
PublisherImageTransport::PublisherImageTransport(
  const rclcpp::NodeOptions & _options)
: Node("publisher_image_transport", _options)
{
  timer_ = this->create_wall_timer(
    33ms,
    std::bind(&PublisherImageTransport::PublishMessage, this));
}

void PublisherImageTransport::SetFilename(
  const std::string & _filename)
{
  this->filename_ = _filename;
}

void PublisherImageTransport::SetCompressType(
  const std::string & _compress_type)
{
  this->compress_type_ = _compress_type;
}

PublisherImageTransport::~PublisherImageTransport()
{
  this->Destroy();
}

void PublisherImageTransport::Destroy()
{
  this->pub.shutdown();
  this->it.reset();
  this->timer_->cancel();
}

void PublisherImageTransport::SetImageSize(int _width, int _height)
{
  std::lock_guard<std::mutex> lock(mutex_);

  std_msgs::msg::Header hdr;
  cv::Rect rect = cv::Rect(0, 0, _width, _height);
  cv::Mat image = cv::Mat(this->image_, rect);
  this->msg_ = cv_bridge::CvImage(hdr, "bgr8", image).toImageMsg();
}

int PublisherImageTransport::Height()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return this->image_.rows;
}

int PublisherImageTransport::Width()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return this->image_.cols;
}

void PublisherImageTransport::PublishMessage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->msg_->header.stamp = this->now();
  this->pub.publish(this->msg_);
  if (this->count == 0) {
    SetCompressJpegParameter();
    SetCompressType();
  }
  count++;
}

int PublisherImageTransport::GetNumberOfImagesPublished()
{
  std::lock_guard<std::mutex> lock(mutex_);
  int result = count;
  count = 0;
  return result;
}

void PublisherImageTransport::Initialize()
{
  this->it = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
  this->pub = this->it->advertise("camera/image", rmw_qos_profile_sensor_data);
  this->image_ = cv::imread(this->filename_, cv::IMREAD_COLOR);
}

void PublisherImageTransport::SetCompressJpeg(int _value)
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->compress_ = _value;
}

void PublisherImageTransport::SetCompressJpegParameter()
{
  if (this->compress_type_ == "jpeg") {
    this->set_parameter(rclcpp::Parameter("camera.image.compressed.jpeg_quality", this->compress_));
  } else if (this->compress_type_ == "png") {
    this->set_parameter(rclcpp::Parameter("camera.image.compressed.png_level", this->compress_));
  } else if (this->compress_type_ == "zstd") {
    this->set_parameter(rclcpp::Parameter("camera.image.zstd.zstd_level", this->compress_));
  }
}

void PublisherImageTransport::SetCompressType()
{
  if (this->compress_type_ == "jpeg" || this->compress_type_ == "png") {
    this->set_parameter(rclcpp::Parameter("camera.image.compressed.format", this->compress_type_));
  }
}

}  // namespace performance_transport
