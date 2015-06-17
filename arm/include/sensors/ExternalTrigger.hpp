/*
 * ExternalTrigger.hpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#ifndef ExternalTrigger_HPP_
#define ExternalTrigger_HPP_

#include "Sensor.hpp"

#define EXTERNAL_TRIGGER_MAX_RATE 200
#define EXTERNAL_TRIGGER_MIN_RATE 1

#define EXTERNAL_TRIGGER_DATA_SIZE 4+4+1
#define EXTERNAL_TRIGGER_BUFFER_LENGTH 60000

class ExternalTrigger : public Sensor {
 public:
  ExternalTrigger(visensor::SensorId::SensorId sensor_id, int ctrl_address, int register_address);
  virtual ~ExternalTrigger();

  virtual SharedRingBuffer* data_mover(){return &data_mover_;};

  virtual int getSensorType(){return visensor::SensorType::EXTERNAL_TRIGGER;};
  virtual void on(uint32_t rate);
  virtual void off();

  void setDefaultConfig(void);

private:
  SharedRingBuffer data_mover_;
};

#endif /* ExternalTrigger_HPP_ */
