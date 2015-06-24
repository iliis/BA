#ifndef __PERF_CNT_HPP__
#define __PERF_CNT_HPP__

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>

class CycleCounter {
 public:
  CycleCounter();
  ~CycleCounter();
  void start();
  long long stop();
 private:
  long perf_event_open(struct perf_event_attr *hw_event, pid_t pid, int cpu,
                       int group_fd, unsigned long flags);

  struct perf_event_attr pe_;
  int fd_;
  bool has_started_;
  long long t_delay_;
};

#endif
