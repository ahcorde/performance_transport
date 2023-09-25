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
#include "performance_transport/utils/ProcessInfo.hpp"

#include <sys/sysinfo.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/times.h>
#include <ctype.h>
#include <unistd.h>
#define LINEBUFFLEN 2048

ProcessInfo::ProcessInfo(uint32_t iProcessId)
{
  mProcessId = iProcessId;

  mJiffiesPerSecond = sysconf(_SC_CLK_TCK);
  mPrevSystemTime = 0;
  mPrevUserTime = 0;
  mPrevKernelTime = 0;

  // calculate total system time from file /proc/stat,
  // the content is like: cpu 7967 550 4155 489328
  FILE * lpFile = fopen("/proc/stat", "r");
  if (lpFile) {
    // skip unnecessary content
    fscanf(lpFile, "cpu");
    uint64_t lTime;
    int lValuesToRead = 4;
    for (int i = 0; i < lValuesToRead; i++) {
      fscanf(lpFile, "%lu", &lTime);
      mPrevSystemTime += lTime;
    }
    fclose(lpFile);
  }

  // get user mode time, kernel mode time, start time
  // for current process from file /proc/[pid]/stat
  char lFileName[256];
  snprintf(lFileName, 256, "/proc/%d/stat", mProcessId);  // NOLINT
  lpFile = fopen(lFileName, "r");
  if (lpFile) {
    // skip unnecessary content
    int lValuesToSkip = 13;
    char lTemp[LINEBUFFLEN];
    for (int i = 0; i < lValuesToSkip; i++) {
      fscanf(lpFile, "%s", lTemp);
    }
    fscanf(lpFile, "%lu %lu", &mPrevUserTime, &mPrevKernelTime);

    // skip unnecessary content
    lValuesToSkip = 6;
    for (int i = 0; i < lValuesToSkip; i++) {
      fscanf(lpFile, "%s", lTemp);
    }
    uint64_t lStartTimeSinceBoot;
    fscanf(lpFile, "%lu", &lStartTimeSinceBoot);
    mStartTimeSinceBoot = lStartTimeSinceBoot / mJiffiesPerSecond;

    fclose(lpFile);
  }
}

ProcessInfo::~ProcessInfo() throw()
{
}

uint32_t ProcessInfo::GetProcessId()
{
  return mProcessId;
}

uint64_t ProcessInfo::GetProcessUptime()
{
  u_int64_t lUptimeInSec = -1;

  struct sysinfo lSysinfo;
  int lReturn = sysinfo(&lSysinfo);

  if (lReturn == 0) {
    lUptimeInSec = lSysinfo.uptime - mStartTimeSinceBoot;
  }

  return lUptimeInSec;
}

double ProcessInfo::GetProcessCPUUsage()
{
  double lCPUUsage = -1;

  uint64_t lCurrSystemTime = 0;
  uint64_t lCurrUserTime = 0;
  uint64_t lCurrKernelTime = 0;

  // calculate total system time from file /proc/stat,
  // the content is like: cpu 7967 550 4155 489328
  FILE * lpFile = fopen("/proc/stat", "r");
  if (lpFile) {
    // skip unnecessary content
    fscanf(lpFile, "cpu");
    uint64_t lTime;
    int lValuesToRead = 4;
    for (int i = 0; i < lValuesToRead; i++) {
      fscanf(lpFile, "%lu", &lTime);
      lCurrSystemTime += lTime;
    }
    fclose(lpFile);
  }

  // get user mode and kernel mode time for current
  // process from file /proc/[pid]/stat
  char lFileName[256];
  snprintf(lFileName, 256, "/proc/%d/stat", mProcessId);  // NOLINT
  lpFile = fopen(lFileName, "r");
  if (lpFile) {
    // skip unnecessary content
    char lTemp[LINEBUFFLEN];
    int lValuesToSkip = 13;
    for (int i = 0; i < lValuesToSkip; i++) {
      fscanf(lpFile, "%s", lTemp);
    }

    fscanf(lpFile, "%lu %lu", &lCurrUserTime, &lCurrKernelTime);
    fclose(lpFile);
  }

  uint64_t lTotalProcess = (lCurrUserTime - mPrevUserTime) +
    (lCurrKernelTime - mPrevKernelTime);
  uint64_t lTotalSystem = lCurrSystemTime - mPrevSystemTime;
  if (lTotalSystem > 0) {
    lCPUUsage = (lTotalProcess * 100.0) / lTotalSystem;
  }

  mPrevSystemTime = lCurrSystemTime;
  mPrevUserTime = lCurrUserTime;
  mPrevKernelTime = lCurrKernelTime;

  return lCPUUsage;
}

double ProcessInfo::GetMemUsed()
{
  return this->mlMemUsed;
}

double ProcessInfo::GetMemAnonUsed()
{
  return this->mlMemAnonUsed;
}

double ProcessInfo::GetMemVmUsed()
{
  return this->mlMemVmUsed;
}

void ProcessInfo::GetProcessMemoryUsed()
{
  mlMemUsed = -1;

  char lFileName[256];
  snprintf(lFileName, 256, "/proc/%d/status", mProcessId);  // NOLINT
  FILE * lpFile = fopen(lFileName, "r");
  char lLineBuf[LINEBUFFLEN];
  auto getValue = [](char * _lLineBuf)
    {
      char * cursor = _lLineBuf + 6;
      /* Get rid of preceding blanks */
      while (!isdigit(*cursor)) {
        cursor++;
      }
      /* Get rid of following blanks */
      char * lNumString = cursor;
      while (isdigit(*cursor)) {
        cursor++;
      }
      *cursor = '\0';
      return lNumString;
    };
  if (lpFile) {
    while (fgets(lLineBuf, LINEBUFFLEN, lpFile)) {
      if (0 == strncmp(lLineBuf, "VmRSS:", 6)) {
        mlMemUsed = atoi(getValue(lLineBuf)) / 1024.0;
      }
      if (0 == strncmp(lLineBuf, "RssAnon:", 6)) {
        mlMemAnonUsed = atoi(getValue(lLineBuf)) / 1024.0;
      }
      if (0 == strncmp(lLineBuf, "VmSize:", 6)) {
        mlMemVmUsed = atoi(getValue(lLineBuf)) / 1024.0;
      }
    }
    fclose(lpFile);
  }
}

uint64_t ProcessInfo::GetProcessThreadCount()
{
  uint64_t lThreadCnt = -1;

  // get number of threads from file /proc/[pid]/stat
  char lFileName[256];
  snprintf(lFileName, 256, "/proc/%d/stat", mProcessId);  // NOLINT
  FILE * lpFile = fopen(lFileName, "r");
  if (lpFile) {
    // skip unnecessary content
    char lTemp[LINEBUFFLEN];
    int lValuesToSkip = 19;
    for (int i = 0; i < lValuesToSkip; i++) {
      fscanf(lpFile, "%s", lTemp);
    }
    fscanf(lpFile, "%lu", &lThreadCnt);
    fclose(lpFile);
  }

  return lThreadCnt;
}
