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

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>


#include "performance_transport/utils/utils.hpp"

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace performance_transport
{
SubscriberImageTransport::SubscriberImageTransport(
  const rclcpp::NodeOptions & _options)
: Node("subscriber_image_transport", _options)
{
}

void SubscriberImageTransport::SetTransportHint(
  const std::string & _transport_hint)
{
  this->transport_hint_ = _transport_hint;
}

void SubscriberImageTransport::SetCompressType(
  const std::string & _compress_type)
{
  this->compress_type_ = _compress_type;
}

SubscriberImageTransport::~SubscriberImageTransport()
{
  if (this->dataCollector_ != nullptr) {
    this->dataCollector_->Close();
  }
  if (this->systemDataCollector != nullptr) {
    this->systemDataCollector->Close();
  }
}

void SubscriberImageTransport::Destroy()
{
  if (this->dataCollector_ != nullptr) {
    this->dataCollector_->Close();
  }

  if (systemDataCollector != nullptr) {
    this->systemDataCollector->Close();
  }
  this->timer_->cancel();

  this->sub.shutdown();
  this->it.reset();
  this->timer_->cancel();
}


bool SubscriberImageTransport::IsFinished()
{
  return this->stop_;
}

void SubscriberImageTransport::checkSubscribers()
{
  auto current_time_stamp_seconds = timeToSec(this->now());
  auto diff = current_time_stamp_seconds - this->last_update;

  if (diff > 1) {
    this->stop_ = true;
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

  this->last_update = current_time_stamp_seconds;

  if (this->size_ != msg->width) {
    this->size_ = msg->width;

    std::string filename = "subscriber_data" + std::string("_") +
      std::to_string(size_) + std::string("_") +
      std::to_string(size_) + std::string("_") +
      this->transport_hint_;

    std::string filenameSystemData = "subscriber_data_cpu_mem" + std::string("_") +
      std::to_string(size_) + std::string("_") +
      std::to_string(size_) + std::string("_") +
      this->transport_hint_;

    if (this->transport_hint_ == "compressed") {
      filename += std::string("_") + this->compress_type_;
      filename += std::string("_") + std::to_string(this->compress_);

      filenameSystemData += std::string("_") + this->compress_type_;
      filenameSystemData += std::string("_") + std::to_string(this->compress_);
    }
    filename += ".csv";
    filenameSystemData += ".csv";

    this->dataCollector_ = std::make_shared<DataCollector>(filename);
    this->dataCollector_->WriteLine("fps, response_t");

    this->systemDataCollector = std::make_shared<SystemDataCollector>(
      filenameSystemData,
      this->get_clock());
    this->timer_ = this->create_wall_timer(
      1s,
      std::bind(&SubscriberImageTransport::checkSubscribers, this));
    this->start = std::chrono::high_resolution_clock::now();
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> elapsed = finish - this->start;
  if (elapsed.count() > 1) {
    double fps = static_cast<double>(this->count_) / static_cast<double>(elapsed.count());
    double response_time = static_cast<double>(this->diff_time_sim_) /
      static_cast<double>(this->count_);
    this->count_ = 0;
    this->diff_time_sim_ = 0;
    this->start = std::chrono::high_resolution_clock::now();
    if (dataCollector_ != nullptr) {
      dataCollector_->WriteLine(
        std::to_string(fps) + "," +
        std::to_string(response_time));
    }
  }
}

void SubscriberImageTransport::Initialize()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    this,
    "publisher_image_transport");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  std::stringstream ss;

  std::string param_name;
  if (this->compress_type_ == "jpeg") {
    param_name = "camera.image.compressed.jpeg_quality";
  } else if (this->compress_type_ == "png") {
    param_name = "camera.image.compressed.png_level";
  } else if (this->compress_type_ == "zstd") {
    param_name = "camera.image.zstd.zstd_level";
  }

  for (
    auto & parameter : parameters_client->get_parameters({param_name}))
  {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string();
    this->compress_ = std::atoi(parameter.value_to_string().c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Parameter: %s", ss.str().c_str());

  this->it = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
  image_transport::TransportHints th(this->shared_from_this().get(), this->transport_hint_);
  this->sub = it->subscribe(
    "camera/image", 1,
    std::bind(&SubscriberImageTransport::imageCallback, this, _1),
    nullptr, &th);

  RCLCPP_INFO(this->get_logger(), "Initialized");
}
}  // namespace performance_transport
