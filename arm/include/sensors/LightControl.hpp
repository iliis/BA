/*
 * LightControl.hpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#ifndef LightControl_HPP_
#define LightControl_HPP_

#include "Sensor.hpp"

class LightControl : public Sensor {
 public:
  LightControl(visensor::SensorId::SensorId sensor_id, int register_address);
  virtual ~LightControl();

  virtual int getSensorType(){return visensor::SensorType::LIGHT_CONTROL;};
  virtual void on();
  virtual void off();

  void setIntensity(int32_t time);
  int32_t getIntensity();

private:
};

#endif /* LightControl_HPP_ */
