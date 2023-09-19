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

#ifndef PERFORMANCE_TRANSPORT__PUBLISHERIMAGETRANSPORT_HPP_
#define PERFORMANCE_TRANSPORT__PUBLISHERIMAGETRANSPORT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <image_transport/image_transport.hpp>

#include <opencv2/core/mat.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace performance_transport
{
class PublisherImageTransport : public rclcpp::Node
{
public:
  PublisherImageTransport(
    const rclcpp::NodeOptions & _options,
    const std::string & _filename);
  void Initialize();
  void PublishMessage();
  void SetImageSize(int _width, int _height);
  int Width();
  int Height();
  void SetCompressJpeg(int _value);
  int GetNumberOfImagesPublished();

private:
  std::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr compress_pub_;

  cv::Mat image_;
  std::string filename_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  std::mutex mutex_;
  int count{0};
  int compress_{0};
};
}  // namespace performance_transport

#endif  // PERFORMANCE_TRANSPORT__PUBLISHERIMAGETRANSPORT_HPP_
