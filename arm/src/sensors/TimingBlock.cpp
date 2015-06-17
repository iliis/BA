/*
 * TimingBlock.cpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#include "sensors/TimingBlock.hpp"
#include <time.h>

static const double time_sync_period = 1; // in seconds
static const double time_sync_timeout = 1; // in mili seconds

TimingBlock::TimingBlock(int register_address)
: Sensor(visensor::SensorId::SENSOR_CLOCK, register_address, 8){
  last_tick_time_ = getTime();
  tick_period_ = 100000.0 * time_sync_period;
}

TimingBlock::~TimingBlock() {
}

void TimingBlock::on() {

  // start counter
  config_registers_.setRegisterValue(TIME_CTRL, 0X01);

  // set to zero
  config_registers_.setRegisterValue(TIME_WR, 0x00);

  // Debug stuff
  printf("========================================\n");

  int val = config_registers_.getRegisterValue(TIME_CTRL);
  printf("TIME_CTRL %d\n", val);

//  printf("testing timer block:\n");
//  for(int i = 0; i<10; i++){
//    val = config_registers_.getRegisterValue(TIME_RD);
//    printf("reg val %d\n", val);
//    nanosleep(&slptm,NULL);
//  }
  printf("========================================\n");
}

void TimingBlock::off() {
  // don't turn off the clock, fpga does not like it
//  config_registers_.setRegisterValue(TIME_CTRL, 0X00);
//  config_registers_.setRegisterValue(TIME_WR, 0x00);
  wait_condition_.notify_one();
}


void TimingBlock::setTime(uint32_t time) {
  config_registers_.setRegisterValue(TIME_WR, time);
}

// returns true if defined period is elapsed
bool TimingBlock::needToSyncTime() {
  if(last_tick_time_ + tick_period_ < getTime())
    return true;
  return false;
}

void TimingBlock::waitForTimeSyncCompletion() {
  boost::system_time const timeout=boost::get_system_time()+ boost::posix_time::milliseconds(time_sync_timeout);

  boost::unique_lock<boost::mutex> lock(mutex_);
  wait_condition_.timed_wait(lock, timeout);
  last_tick_time_ = getTime();
}

void TimingBlock::signalSyncCompleted() {
  wait_condition_.notify_one();
}

uint32_t TimingBlock::getTime() {
  return config_registers_.getRegisterValue(TIME_RD);
}
