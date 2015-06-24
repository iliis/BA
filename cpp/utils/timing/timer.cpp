// Copyright 2013 Motorola Mobility LLC. Part of the Trailmix project.
// CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
#include <algorithm>
#include <ostream>  //NOLINT
#include <sstream>
#include <stdio.h>
#include <string>
#include <iomanip>      // std::setw

#include "timer.hpp"

namespace timing {

Timing& Timing::Instance() {
  static Timing t;
  return t;
}

Timing::Timing() : maxTagLength_(0) { }

Timing::~Timing() { }

// Static functions to query the timers:
size_t Timing::GetHandle(std::string const& tag) {
  // Search for an existing tag.
  map_t::iterator i = Instance().tagMap_.find(tag);
  if (i == Instance().tagMap_.end()) {
    // If it is not there, create a tag.
    size_t handle = Instance().timers_.size();
    Instance().tagMap_[tag] = handle;
    Instance().timers_.push_back(TimerMapValue());
    // Track the maximum tag length to help printing a table of timing values
    // later.
    Instance().maxTagLength_ = std::max(Instance().maxTagLength_, tag.size());
    return handle;
  } else {
    return i->second;
  }
}

std::string Timing::GetTag(size_t handle) {
  std::string tag = "";

  // Perform a linear search for the tag.
  for (typename map_t::value_type current_tag : Instance().tagMap_) {
    if (current_tag.second == handle) {
      return current_tag.first;
    }
  }
  return tag;
}

// Class functions used for timing.
Timer::Timer(size_t handle, bool construct_stopped)
    : timing_(false),
      handle_(handle) {
init();

  if (!construct_stopped)
    Start();
}

void Timer::init(){
#if defined(USE_RDTSC) && defined(__ARM_ARCH_7A__)
  use_event_counter_ = false;
  enable_ccnt();
#elif USE_PERF_COUNTER
  memset(&pe_, 0, sizeof(struct perf_event_attr));
  pe_.type = PERF_TYPE_HARDWARE;
  pe_.size = sizeof(struct perf_event_attr);
  pe_.config = PERF_COUNT_HW_CPU_CYCLES;  //     Total cycles. Be wary of what happens during CPU frequency scaling. (http://web.eece.maine.edu/~vweaver/projects/perf_events/perf_event_open.html)
  pe_.disabled = 1;
  pe_.exclude_kernel = 0;
  pe_.exclude_hv = 0;

  const static pid_t pid = 0;
  const static int cpu = -1;
  const static int group_fd = -1;
  const static unsigned long flags = 0;

  fd_ = syscall(__NR_perf_event_open, &pe_, pid, cpu, group_fd, flags);
  if (fd_ == -1) {
    fprintf(stderr, "[perf counter] Error opening leader %llx\n", pe_.config);
    exit(EXIT_FAILURE);
  }

  //start();
  t_delay_ = 0;//stop(); //TODO(burrimi): Estimate delay.
#endif
}

Timer::Timer(std::string const& tag, bool construct_stopped)
    : timing_(false),
      handle_(Timing::GetHandle(tag)) {
  init();

  if (!construct_stopped)
    Start();
}

#if defined(USE_RDTSC) && defined(__ARM_ARCH_7A__)
Timer::Timer(std::string const& tag, unsigned int counter_id, PMU_EVENT event, bool construct_stopped)
    : timing_(false),
      handle_(Timing::GetHandle(tag)),
      use_event_counter_(true),
      counter_nr_(counter_id),
      event_id_(event){

  if(use_event_counter_) {
    enable_pmu();              // Enable the PMU
    //  reset_pmn();               // Reset the configurable counters
    pmn_config(counter_nr_, static_cast<unsigned int>(event_id_));       // Configure counter 0 to count event code 0x03
    //  enable_pmn(counter_nr);             // Enable counter
  } else {
    enable_ccnt();
  }

  if (!construct_stopped)
    Start();
}
#else
Timer::Timer(std::string const& tag, unsigned int counter_id, PMU_EVENT event, bool construct_stopped)
: timing_(false),
  handle_(Timing::GetHandle(tag)){
  init();
  if (!construct_stopped)
    Start();
}
#endif

Timer::~Timer() {
  if (IsTiming())
    Stop();
#ifdef USE_PERF_COUNTER
  close(fd_);
#endif
}

bool Timer::IsTiming() const {
  return timing_;
}

size_t Timer::GetHandle() const {
  return handle_;
}

void Timing::AddTime(size_t handle, double seconds) {
  timers_[handle].acc_.Add(seconds);
}

void Timing::AddCycles(size_t handle, double cycles) {
  timers_[handle].acc_.Add(cycles);
}

double Timing::GetTotalSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Sum();
}
double Timing::GetTotalSeconds(std::string const& tag) {
  return GetTotalSeconds(GetHandle(tag));
}
double Timing::GetMeanSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Mean();
}
double Timing::GetMeanSeconds(std::string const& tag) {
  return GetMeanSeconds(GetHandle(tag));
}
size_t Timing::GetNumSamples(size_t handle) {
  return Instance().timers_[handle].acc_.TotalSamples();
}
size_t Timing::GetNumSamples(std::string const& tag) {
  return GetNumSamples(GetHandle(tag));
}
double Timing::GetVarianceSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.LazyVariance();
}
double Timing::GetVarianceSeconds(std::string const& tag) {
  return GetVarianceSeconds(GetHandle(tag));
}
double Timing::GetMinSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Min();
}
double Timing::GetMinSeconds(std::string const& tag) {
  return GetMinSeconds(GetHandle(tag));
}
double Timing::GetMaxSeconds(size_t handle) {
  return Instance().timers_[handle].acc_.Max();
}
double Timing::GetMaxSeconds(std::string const& tag) {
  return GetMaxSeconds(GetHandle(tag));
}

double Timing::GetHz(size_t handle) {
  return 1.0 / Instance().timers_[handle].acc_.RollingMean();
}

double Timing::GetHz(std::string const& tag) {
  return GetHz(GetHandle(tag));
}

std::string Timing::SecondsToTimeString(double seconds) {
  double secs = fmod(seconds, 60);
  int minutes = (seconds / 60);
  int hours = (seconds / 3600);
  minutes -= (hours * 60);

char buffer[256];
snprintf(buffer, sizeof(buffer),
#ifdef BRISK_TIMING_SHOW_HOURS
"%02d:"
#endif
#ifdef BRISK_TIMING_SHOW_MINUTES
"%02d:"
#endif
"%09.6f",
#ifdef BRISK_TIMING_SHOW_HOURS
hours,
#endif
#ifdef BRISK_TIMING_SHOW_MINUTES
minutes,
#endif
secs);
return buffer;
}

void Timing::Print(std::ostream& out) {  //NOLINT
  map_t& tagMap = Instance().tagMap_;

  if (tagMap.empty()) {
    return;
  }
#ifdef USE_RDTSC
  out << "Timing - RDTSC cycles\n";
#elif USE_PERF_COUNTER
  out << "Timing - perf_cnt cycles\n";
#else
  out << "Timing\n";
#endif
  out << "-----------\n";
  out << "name   samples total (mean +/- stddev) [min max]\n";
  for (typename map_t::value_type t : tagMap) {
    size_t i = t.second;
    out.width((std::streamsize) Instance().maxTagLength_);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << t.first << "\t";
    out.width(7);

    out.setf(std::ios::right, std::ios::adjustfield);
    out << GetNumSamples(i) << "\t";
    if (GetNumSamples(i) > 0) {
#ifdef USE_RDTSC
      out << std::scientific;
      out.precision(2);
      out << GetTotalSeconds(i) << "\t";
      double meansec = GetMeanSeconds(i);
      double stddev = sqrt(GetVarianceSeconds(i));
      out << "(" << meansec << " +/- ";
      out << stddev << ")\t";

      double minsec = GetMinSeconds(i);
      double maxsec = GetMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << minsec << ", " << maxsec << "]";
#elif USE_PERF_COUNTER
      out << GetTotalSeconds(i) << "\t";
      double meansec = GetMeanSeconds(i);
      double stddev = sqrt(GetVarianceSeconds(i));
      out << "(" << meansec << " +/- ";
      out << stddev << ")\t";

      double minsec = GetMinSeconds(i);
      double maxsec = GetMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << minsec << ", " << maxsec << "]";
#else
      out << SecondsToTimeString(GetTotalSeconds(i)) << "\t";
      double meansec = GetMeanSeconds(i);
      double stddev = sqrt(GetVarianceSeconds(i));
      out << "(" << SecondsToTimeString(meansec) << " +/- ";
      out << SecondsToTimeString(stddev) << ")\t";

      double minsec = GetMinSeconds(i);
      double maxsec = GetMaxSeconds(i);

      // The min or max are out of bounds.
      out << "[" << SecondsToTimeString(minsec) << ","
          << SecondsToTimeString(maxsec) << "]";
#endif
    }
    out << std::endl;
  }
  for (typename map_t::value_type t : tagMap) {
    size_t i = t.second;
    out << GetTotalSeconds(i)/10.0 << "\n";
  }
  out << std::endl;

}
std::string Timing::Print() {
  std::stringstream ss;
  Print(ss);
  return ss.str();
}

void Timing::Reset() {
  Instance().tagMap_.clear();
}

}  // namespace timing
