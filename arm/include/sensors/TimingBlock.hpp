/*
 * TimingBlock.hpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#ifndef TimingBlock_HPP_
#define TimingBlock_HPP_

#include <boost/thread/shared_mutex.hpp>

#include "Sensor.hpp"

class TimingBlock : public Sensor {
 public:
  TimingBlock(int register_address);
  virtual ~TimingBlock();

  virtual int getSensorType(){return visensor::SensorType::TIMING_BLOCK;};
  virtual void on();
  virtual void off();

  // returns true if minimal time between two syncs is over
  bool needToSyncTime();
  // returns after a time sync is completed
  void waitForTimeSyncCompletion();
  // must be called after sync to release waitForTimeSyncCompletion()
  void signalSyncCompleted();

  virtual void setTime(uint32_t time);
  virtual uint32_t getTime();

private:
  uint32_t last_tick_time_;
  uint32_t tick_period_;
  boost::condition_variable wait_condition_;
  boost::mutex mutex_;
};

#endif /* TimingBlock_HPP_ */
