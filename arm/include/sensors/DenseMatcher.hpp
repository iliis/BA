/*
 * DenseMatcher.hpp
 *
 *  Created on: Jul 15, 2013
 *      Author: pascal
 */

#ifndef DenseMatcher_HPP_
#define DenseMatcher_HPP_

#include <string>

#include "fpga/UnifiedBusInterface.hpp"
#include "sensors/CamBase.hpp"

class DenseMatcher : public CamBase {
 public:
  DenseMatcher(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress);
  virtual ~DenseMatcher();

  static bool checkPresence(int bus_address);
  virtual int getSensorType() {
    return visensor::SensorType::DENSE_MATCHER;
  }
  ;
  virtual void on(uint32_t rate);
  void off();

 private:
};

#endif /* DenseMatcher_HPP_ */
