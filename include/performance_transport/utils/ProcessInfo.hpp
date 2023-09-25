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

#ifndef PERFORMANCE_TRANSPORT__UTILS__PROCESSINFO_HPP_
#define PERFORMANCE_TRANSPORT__UTILS__PROCESSINFO_HPP_

#include <cstdint>

class ProcessInfo
{
public:
  explicit ProcessInfo(uint32_t iProcessId);
  ~ProcessInfo() throw();

  uint32_t GetProcessId();
  uint64_t GetProcessUptime();
  double GetProcessCPUUsage();
  void GetProcessMemoryUsed();
  uint64_t GetProcessThreadCount();

  double GetMemUsed();
  double GetMemAnonUsed();
  double GetMemVmUsed();

private:
  uint32_t mProcessId;
  int32_t mJiffiesPerSecond;
  uint64_t mStartTimeSinceBoot;
  uint64_t mPrevSystemTime;
  uint64_t mPrevUserTime;
  uint64_t mPrevKernelTime;
  double mlMemUsed;
  double mlMemAnonUsed;
  double mlMemVmUsed;
};

#endif  // PERFORMANCE_TRANSPORT__UTILS__PROCESSINFO_HPP_
