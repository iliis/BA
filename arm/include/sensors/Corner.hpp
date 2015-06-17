/*
 * Corners.hpp
 *
 *  Created on: Jul 15, 2013
 *      Author: pascal
 */

#ifndef Corner_HPP_
#define Corner_HPP_

#include <string>

#include "sensors/CamBase.hpp"

class Corner : public CamBase {
 public:
  Corner(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress);
  virtual ~Corner();

  virtual int getSensorType() {
    return visensor::SensorType::CORNER_MT9V034;
  }
  ;
  virtual void on(uint32_t rate);
  void off();
};

#endif /* Corner_HPP_ */
