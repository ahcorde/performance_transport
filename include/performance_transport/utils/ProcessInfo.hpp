#ifndef ProcessInfo_h
#define ProcessInfo_h

#ifdef _WIN32
#include <windows.h>
#endif

class ProcessInfo
{
public:
  ProcessInfo(unsigned int iProcessId);
  ~ProcessInfo() throw();

  unsigned int GetProcessId();
  unsigned long long GetProcessUptime();
  double GetProcessCPUUsage();
  void GetProcessMemoryUsed();
  unsigned long GetProcessThreadCount();

  double GetMemUsed();
  double GetMemAnonUsed();
  double GetMemVmUsed();

private:
  unsigned int mProcessId;
  long mJiffiesPerSecond;
  unsigned long long mStartTimeSinceBoot;
  unsigned long long mPrevSystemTime;
  unsigned long long mPrevUserTime;
  unsigned long long mPrevKernelTime;
  double mlMemUsed;
  double mlMemAnonUsed;
  double mlMemVmUsed;
};

#endif
