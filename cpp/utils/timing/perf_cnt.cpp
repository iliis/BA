#include "perf_cnt.h"

CycleCounter::CycleCounter()
    : has_started_(false),
      t_delay_(0) {

  memset(&pe_, 0, sizeof(struct perf_event_attr));
  pe_.type = PERF_TYPE_HARDWARE;
  pe_.size = sizeof(struct perf_event_attr);
  pe_.config = PERF_COUNT_HW_CPU_CYCLES;  //     Total cycles. Be wary of what happens during CPU frequency scaling. (http://web.eece.maine.edu/~vweaver/projects/perf_events/perf_event_open.html)
  pe_.disabled = 1;
  pe_.exclude_kernel = 1;
  pe_.exclude_hv = 1;

  fd_ = perf_event_open(&pe_, 0, -1, -1, 0);
  if (fd_ == -1) {
    fprintf(stderr, "Error opening leader %llx\n", pe_.config);
    exit(EXIT_FAILURE);
  }

  start();
  t_delay_ = stop();
  //printf("t_delay %.0f\n", t_delay_*1.0f);
}

CycleCounter::~CycleCounter() {
  close(fd_);
}

void CycleCounter::start() {
  if (has_started_) {
    printf("Already started\n");
    return;
  }
  has_started_ = true;
  ioctl(fd_, PERF_EVENT_IOC_RESET, 0);
  ioctl(fd_, PERF_EVENT_IOC_ENABLE, 0);
}

long long CycleCounter::stop() {
  long long count;
  ioctl(fd_, PERF_EVENT_IOC_DISABLE, 0);
  read(fd_, &count, sizeof(long long));
  if (!has_started_) {
    printf("Didnt start yet, returning 0\n");
    count = 0;
  }
  has_started_ = false;
  return count - t_delay_;
}

long CycleCounter::perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                                   int cpu, int group_fd, unsigned long flags) {
  int ret;

  ret = syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
  return ret;
}
