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

#include "performance_transport/image_transport/SubscriberImageTransport.hpp"
#include "performance_transport/utils/utils.hpp"

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

using std::placeholders::_1;
namespace performance_transport
{
SubscriberImageTransport::SubscriberImageTransport(
  const rclcpp::NodeOptions & _options,
  const std::string & _transport_hint)
: Node("subscriber_image_transport", _options), transport_hint_(_transport_hint)
{
}

SubscriberImageTransport::~SubscriberImageTransport()
{
  this->dataCollector_->Close();
}

void SubscriberImageTransport::compress_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> elapsed = finish - this->start;
  double fps = static_cast<double>(count_) / static_cast<double>(elapsed.count());
  double response_time = static_cast<double>(diff_time_sim_) / static_cast<double>(count_);
  if (dataCollector_ != nullptr) {
    dataCollector_->WriteLine(
      std::to_string(fps) + "," +
      std::to_string(this->compress_) + "," +
      std::to_string(response_time));
  }
  this->count_ = 0;
  this->diff_time_sim_ = 0;
  this->compress_ = msg->data;
  this->start = std::chrono::high_resolution_clock::now();
  if (this->compress_ == -5) {
    dataCollector_->Close();
    systemDataCollector->Close();
    exit(0);
  }
}

void SubscriberImageTransport::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  this->count_++;
  cv_bridge::toCvShare(msg, "bgr8")->image;
  auto msg_time_stamp_seconds = timeToSec(msg->header.stamp);
  auto current_time_stamp_seconds = timeToSec(this->now());

  this->diff_time_sim_ += current_time_stamp_seconds - msg_time_stamp_seconds;

  if (this->size_ != msg->width) {
    this->size_ = msg->width;
    this->dataCollector_ = std::make_shared<DataCollector>(
      "subscriber_data" + std::string("_") +
      std::to_string(size_) + std::string("_") +
      std::to_string(size_) + ".csv");
    this->start = std::chrono::high_resolution_clock::now();

    systemDataCollector = std::make_shared<SystemDataCollector>(
      "subscriber_data_cpu_mem" + std::string("_") +
      std::to_string(size_) + std::string("_") +
      std::to_string(size_) + ".csv",
      this->get_clock());
  }
}

void SubscriberImageTransport::Initialize()
{
  this->it = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
  image_transport::TransportHints th(this->shared_from_this().get(), this->transport_hint_);
  this->sub = it->subscribe(
    "camera/image", 100,
    std::bind(&SubscriberImageTransport::imageCallback, this, _1),
    nullptr, &th);
  this->subscription_compress_ = this->create_subscription<std_msgs::msg::Int32>(
    "compress", 10,
    std::bind(&SubscriberImageTransport::compress_callback, this, _1));
}
}  // namespace performance_transport
