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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/times.h>

#include <memory>
#include <string>

#include "performance_transport/point_cloud_transport/PublisherPointCloudTransport.hpp"

#include <rclcpp/rclcpp.hpp>

#include "performance_transport/utils/ProcessInfo.hpp"

int parseLine(char * line)
{
  // This assumes that a digit will be found and the line ends in " Kb".
  int i = strlen(line);
  const char * p = line;
  while (*p < '0' || *p > '9') {p++;}
  line[i - 3] = '\0';
  i = atoi(p);
  return i;
}

int getVirtualValue()  //  Note: this value is in KB!
{
  FILE * file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL) {
    if (strncmp(line, "VmSize:", 7) == 0) {
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  return result;
}
int getRSSAnonValue()  //  Note: this value is in KB!
{
  FILE * file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL) {
    if (strncmp(line, "RssAnon:", 7) == 0) {
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  return result;
}

void statistics()
{
  rclcpp::WallRate loop_rate(1);
  ProcessInfo pinfo(getpid());
  while (rclcpp::ok()) {
    loop_rate.sleep();
    pinfo.GetProcessMemoryUsed();
    std::cout << "GetProcessUptime() " << pinfo.GetProcessUptime() << std::endl;
    std::cout << "GetProcessCPUUsage() " << pinfo.GetProcessCPUUsage() << std::endl;
    std::cout << "GetProcessMemoryUsed() "
              << pinfo.GetMemUsed() << " Anon: "
              << pinfo.GetMemAnonUsed() << " VM: "
              << pinfo.GetMemVmUsed() << std::endl;
    std::cout << "GetProcessThreadCount() " << pinfo.GetProcessThreadCount() << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  if (argc < 2) {
    std::cout << "Usage publish_point_cloud <filename> " << std::endl;
    return -1;
  }
  std::string filename = argv[1];

  std::shared_ptr<performance_transport::PublisherPointCloudTransport> ppc =
    std::make_shared<performance_transport::PublisherPointCloudTransport>(options, filename);
  ppc->Initialize();

  rclcpp::WallRate loop_rate(30);

  std::thread t1(statistics);

  while (rclcpp::ok()) {
    loop_rate.sleep();

    rclcpp::spin_some(ppc);
  }

  rclcpp::shutdown();

  return 0;
}
