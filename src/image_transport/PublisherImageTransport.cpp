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
  const rclcpp::NodeOptions & _options,
  const std::string & _filename)
: Node("publisher_image_transport", _options),
  filename_(_filename)
{
  this->compress_pub_ = this->create_publisher<std_msgs::msg::Int32>("compress", 10);

  timer_ = this->create_wall_timer(
    33ms,
    std::bind(&PublisherImageTransport::PublishMessage, this));
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
  this->pub = this->it->advertise("camera/image", 1);
  this->image_ = cv::imread(this->filename_, cv::IMREAD_COLOR);
}

void PublisherImageTransport::SetCompressJpeg(int _value)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    this->compress_ = _value;
  }

  std_msgs::msg::Int32 msg_compress;
  msg_compress.data = _value;
  this->compress_pub_->publish(msg_compress);

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this->shared_from_this());

  // Set several different types of parameters.
  auto set_parameters_results = parameters_client->set_parameters(
    {
      rclcpp::Parameter("camera.image.compressed.jpeg_quality", _value),
    });
  // Check to see if they were set.
  for (auto & result : set_parameters_results) {
    if (!result.successful) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
    }
  }
}
}  // namespace performance_transport
