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

#ifndef PERFORMANCE_TRANSPORT__IMAGE_TRANSPORT__SUBSCRIBERIMAGETRANSPORT_HPP_
#define PERFORMANCE_TRANSPORT__IMAGE_TRANSPORT__SUBSCRIBERIMAGETRANSPORT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

#include "performance_transport/utils/DataCollector.hpp"
#include "performance_transport/utils/SystemDataCollector.hpp"

namespace performance_transport
{
class SubscriberImageTransport : public rclcpp::Node
{
public:
  SubscriberImageTransport(
    const rclcpp::NodeOptions & _options);
  ~SubscriberImageTransport();
  void Initialize();
  void checkSubscribers();

  void SetTransportHint(const std::string & _transport_hint);
  void SetCompressType(const std::string & _compress_type);
private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  std::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Subscriber sub;
  std::string transport_hint_;
  std::mutex mutex_;
  int count_{0};
  unsigned int size_{0};
  int compress_{95};
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<DataCollector> dataCollector_;
  std::chrono::time_point<std::chrono::system_clock> start;

  std::string compress_type_{};

  std::shared_ptr<SystemDataCollector> systemDataCollector;

  double diff_time_sim_{0};
  double last_update{0};
};
}  // namespace performance_transport

#endif  // PERFORMANCE_TRANSPORT__IMAGE_TRANSPORT__SUBSCRIBERIMAGETRANSPORT_HPP_
