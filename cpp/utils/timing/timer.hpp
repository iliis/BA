#ifndef BRISK_TIMING_TIMER_H_
#define BRISK_TIMING_TIMER_H_

#include <algorithm>
#include <chrono>
#include <limits>
#include <map>
#include <string>
#include <vector>

//#define NO_TIMER

//#define USE_PERF_COUNTER 1
#define USE_RDTSC 1
//#define __ARM_ARCH_7A__ 1

#ifdef USE_PERF_COUNTER
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>
#elif defined(USE_RDTSC) && defined(__ARM_ARCH_7A__)
#include "v7_pmu.hpp"
#elif  USE_RDTSC
#include "rdtsc.h"
#endif

namespace timing {

static const double kNumSecondsPerNanosecond = 1.e-9;

template<typename T, typename Total, int N>
class Accumulator {
 public:
  Accumulator()
      : window_samples_(0), totalsamples_(0),
        sum_(0), window_sum_(0),
        min_(std::numeric_limits <T> ::max()),
        max_(std::numeric_limits <T> ::min()) { }

  void Add(T sample) {
    if (window_samples_ < N) {
      samples_[window_samples_++] = sample;
      window_sum_ += sample;
    } else {
      T& oldest = samples_[window_samples_++ % N];
      window_sum_ += sample - oldest;
      oldest = sample;
    }
    sum_ += sample;
    ++totalsamples_;
    if (sample > max_) {
      max_ = sample;
    }
    if (sample < min_) {
      min_ = sample;
    }
  }

  int TotalSamples() const {
    return totalsamples_;
  }

  double Sum() const {
    return sum_;
  }

  double Mean() const {
    return sum_ / totalsamples_;
  }

  double RollingMean() const {
    return window_sum_ / std::min(window_samples_, N);
  }

  double Max() const {
    return max_;
  }

  double Min() const {
    return min_;
  }

  double LazyVariance() const {
    if (window_samples_ == 0) {
      return 0.0;
    }
    double var = 0;
    double mean = RollingMean();
    for (int i = 0; i < std::min(window_samples_, N); ++i) {
      var += (samples_[i] - mean) * (samples_[i] - mean);
    }
    var /= std::min(window_samples_, N);
    return var;
  }

 private:
  T samples_[N];
  int window_samples_;
  int totalsamples_;
  Total sum_;
  Total window_sum_;
  T min_;
  T max_;
};

struct TimerMapValue {
  TimerMapValue() { }

  // Create an accumulator with specified window size.
  Accumulator<double, double, 50> acc_;
};


class Timing {
 public:
  typedef std::map<std::string, size_t> map_t;
  friend class Timer;
  // Definition of static functions to query the timers.
  static size_t GetHandle(std::string const& tag);
  static std::string GetTag(size_t handle);
  static double GetTotalSeconds(size_t handle);
  static double GetTotalSeconds(std::string const& tag);
  static double GetMeanSeconds(size_t handle);
  static double GetMeanSeconds(std::string const& tag);
  static size_t GetNumSamples(size_t handle);
  static size_t GetNumSamples(std::string const& tag);
  static double GetVarianceSeconds(size_t handle);
  static double GetVarianceSeconds(std::string const& tag);
  static double GetMinSeconds(size_t handle);
  static double GetMinSeconds(std::string const& tag);
  static double GetMaxSeconds(size_t handle);
  static double GetMaxSeconds(std::string const& tag);
  static double GetHz(size_t handle);
  static double GetHz(std::string const& tag);
  static void Print(std::ostream& out);  //NOLINT
  static std::string Print();
  static std::string SecondsToTimeString(double seconds);
  static void Reset();
  static const map_t& GetTimers() {return Instance().tagMap_;}

 private:
  void AddTime(size_t handle, double seconds);
  void AddCycles(size_t handle, double cycles);

  static Timing& Instance();

  Timing();
  ~Timing();

  typedef std::vector<TimerMapValue> list_t;

  list_t timers_;
  map_t tagMap_;
  size_t maxTagLength_;
};

// A class that has the timer interface but does nothing. Swapping this in in
// place of the Timer class (say with a typedef) should allow one to disable
// timing. Because all of the functions are inline, they should just disappear.
class DummyTimer {
 public:
  DummyTimer(size_t /*handle*/, bool construct_stopped = false) {
    static_cast<void>(construct_stopped);
  }
  DummyTimer(std::string const& /*tag*/, bool construct_stopped = false) {
    static_cast<void>(construct_stopped);
  }
  ~DummyTimer() { }

  void Start() { }
  void Stop() { }
  bool IsTiming() {
    return false;
  }
};

enum class PMU_EVENT{
  INSTRUCTION_CACHE_MISS=0X01,
  DATA_CACHE_MISS=0X03,
  DATA_READ=0X06,
  DATA_WRITE=0X07,
  FLOP=0x73,
  TLB_MISS_STALL=0x62,
  NEON_INSTRUCTIONS=0x74
};

class Timer {
 public:
  Timer(size_t handle, bool construct_stopped = false);
  Timer(std::string const& tag, bool construct_stopped = false);
  Timer(std::string const& tag, unsigned int counter_id, PMU_EVENT event, bool construct_stopped = false);
  ~Timer();

  inline void Start() {
#ifdef NO_TIMER
    return;
#endif
    timing_ = true;
  #if defined(USE_RDTSC) && defined(__ARM_ARCH_7A__)
    if(use_event_counter_) {
      start_ = read_pmn(counter_nr_);
      enable_pmn(counter_nr_);
    } else {
      start_ = read_ccnt();
    }
  #elif USE_RDTSC
    CPUID(); RDTSC(start_);
  #elif USE_PERF_COUNTER
    ioctl(fd_, PERF_EVENT_IOC_RESET, 0);
    ioctl(fd_, PERF_EVENT_IOC_ENABLE, 0);
  #else
    time_ = std::chrono::system_clock::now();
  #endif
  }

  inline void Stop() {
#ifdef NO_TIMER
    return;
#endif
  #if defined(USE_RDTSC) && defined(__ARM_ARCH_7A__)
    if(use_event_counter_) {
      end_ = read_pmn(counter_nr_);
      disable_pmn(counter_nr_);
    } else {
      end_ = read_ccnt();
    }
    double counts = (static_cast<double>(end_ - start_));
    disable_pmu();
    Timing::Instance().AddCycles(handle_, counts);
    enable_pmu();
  #elif USE_RDTSC
    RDTSC(end_); CPUID();
    double cycles = (static_cast<double>(COUNTER_DIFF(end_, start_)));
    Timing::Instance().AddCycles(handle_, cycles);
  #elif USE_PERF_COUNTER
    long long count;
    ioctl(fd_, PERF_EVENT_IOC_DISABLE, 0);
    read(fd_, &count, sizeof(long long));
  const double cycles = double(count - t_delay_);
  Timing::Instance().AddCycles(handle_, cycles);
  #else
    std::chrono::time_point <std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    double dt = static_cast<double>(std::chrono::duration_cast
        <std::chrono::nanoseconds> (now - time_).count())
        * kNumSecondsPerNanosecond;
    Timing::Instance().AddTime(handle_, dt);
  #endif
    timing_ = false;
//    printf("Stop count: %f\n", cycles);
  }
  bool IsTiming() const;
  size_t GetHandle() const;
 private:
  void init();

#if defined(USE_RDTSC) && defined(__ARM_ARCH_7A__)
  uint32_t start_, end_;
  uint32_t counter_nr_;
  PMU_EVENT event_id_;
  bool use_event_counter_;
#elif USE_RDTSC
  tsc_counter start_, end_;
#elif USE_PERF_COUNTER
  long perf_event_open(struct perf_event_attr *hw_event, pid_t pid, int cpu,
                       int group_fd, unsigned long flags);

  struct perf_event_attr pe_;
  int fd_;
  bool has_started_;
  long long t_delay_;
#else
  std::chrono::time_point<std::chrono::system_clock> time_;
#endif
  bool timing_;
  size_t handle_;

};

#if ENABLE_BRISK_TIMING
typedef Timer DebugTimer;
#else
typedef DummyTimer DebugTimer;
#endif

}  // namespace timing
#endif  // BRISK_TIMING_TIMER_H_
