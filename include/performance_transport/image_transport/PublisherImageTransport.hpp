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

#ifndef PERFORMANCE_TRANSPORT__IMAGE_TRANSPORT__PUBLISHERIMAGETRANSPORT_HPP_
#define PERFORMANCE_TRANSPORT__IMAGE_TRANSPORT__PUBLISHERIMAGETRANSPORT_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <image_transport/image_transport.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rosbag2_cpp/reader.hpp>

#include <std_msgs/msg/int32.hpp>

namespace performance_transport
{
class PublisherImageTransport : public rclcpp::Node
{
public:
  PublisherImageTransport(
    const rclcpp::NodeOptions & _options);
  ~PublisherImageTransport();
  void Initialize();
  void PublishMessage();
  void SetImageSize(int _width, int _height);
  int Width();
  int Height();
  void SetCompressJpeg(int _value);
  void SetCompressJpegParameter();
  int GetNumberOfImagesPublished();
  void SetCompressType();
  void SetFilename(const std::string & _filename);
  void SetRosBag(const std::string & _rosbag_topic);
  void SetCompressType(const std::string & _compress_type);
  void Destroy();

private:
  std::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher pub;

  cv::Mat image_;
  std::string filename_;
  std::string rosbag_topic_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  std::mutex mutex_;
  int count{0};
  int compress_{0};
  std::string compress_type_;
  cv::VideoCapture cap;

  rosbag2_cpp::readers::SequentialReader reader;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
};
}  // namespace performance_transport

#endif  // PERFORMANCE_TRANSPORT__IMAGE_TRANSPORT__PUBLISHERIMAGETRANSPORT_HPP_
