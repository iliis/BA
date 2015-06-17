/*
 * IrqQueues.hpp
 *
 *  Created on: Dec 18, 2013
 *      Author: schneith
 */

#ifndef IRQQUEUES_HPP_
#define IRQQUEUES_HPP_

#include <queue>

#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#include "sensors/Sensor.hpp"

#include "fpga/FpgaConstants.hpp"



class IrqInterface;

template<typename T>
class ConcurrentQueue {
 private:
  std::queue<T> q_;
  boost::mutex m_;

 public:

  ConcurrentQueue(const ConcurrentQueue<int> *parent_queue) :
      parent_queue(parent_queue) {};

  ConcurrentQueue() : parent_queue(NULL) {};

  void push(const T& data) {
    boost::lock_guard<boost::mutex> l(m_);
    q_.push(data);
  }

  //if the queue is empty and we pop, we just wait for data!!
  T pop() {
    boost::mutex::scoped_lock l(m_);

    //TODO(schneith): debug remove
    if (q_.size() == 0) {
      std::cout << "Tried to pop empty queue!!" << std::endl;
    }

    T res = q_.front();
    q_.pop();
    return res;
  }

  //checks queue is empty
  bool empty() {
    //TODO(schneith): do we need to lock here?
    boost::mutex::scoped_lock l(m_);
    return q_.size() == 0;
  }

  //return size of queue
  size_t size() {
    //TODO(schneith): do we need to lock here?
    boost::mutex::scoped_lock l(m_);
    return q_.size();
  }

  void clear() {
    std::queue<int> empty;
    std::swap(q_, empty);
  }

  //returns current queue and clear it
  std::queue<T> pop_all() {
    boost::mutex::scoped_lock l(m_);

    std::queue<T> queue;  //empty here
    std::swap(queue, q_);

    return queue;
  }

  //returns current queue and clear it
  std::queue<T> clone() {
    boost::mutex::scoped_lock l(m_);

    std::queue<T> queue = q_;
    return queue;
  }

  //holds the queue with the next higher priority or is NULL
  ConcurrentQueue<int> *parent_queue;
};



class IrqScheduler {


 private:
  boost::mutex m_processIrq_;
  boost::condition_variable cv_irq_received_;
  ConcurrentQueue<int> irqQueue_[3];

  // connection to kernel driver
  IrqInterface *irq_;

  //process irqs?
  bool active_;

  void streamMeasurement(size_t sensor_id);
  void processIrqPrioritizedQueues(ConcurrentQueue<int> *irq_queue);
  void processIrqQueues();
  void dumpQueues();


  // map from (irqNr, sensorNr)
  std::map<int, int> irqSensor =
  {
	  {FPGA_CONSTANTS::IRQ::IMU0, 0},  //Imu0
	  {FPGA_CONSTANTS::IRQ::CAM0, 1}, //Cam0
      {FPGA_CONSTANTS::IRQ::CAM1, 2}, //Cam1
      {FPGA_CONSTANTS::IRQ::CAM2, 3}, //Cam2
      {FPGA_CONSTANTS::IRQ::CAM3, 4} //Cam3
  };

  //TODO(schneith): find better type for this, so we can do auto init
  enum irqQueues {
    HIGH_PRIORITY,
    MED_PRIORITY,
    LOW_PRIORITY,

    //size
    NUM_IRQQUEUES
  };

  std::vector<Sensor*> sensor_;

  public:
    void setActive(bool onoff);
    void irqCallback(int irqNr);
    void printIrqStatistics();

    IrqScheduler(std::vector<Sensor*> sensor);
    ~IrqScheduler();
};


#endif /* IRQQUEUES_HPP_ */
