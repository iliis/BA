/*
 * ADIS16488.hpp
 *
 *  Created on: Apr 2, 2014
 *      Author: pascal
 */

#ifndef ADIS16488_HPP_
#define ADIS16488_HPP_

#include "ImuBase.hpp"

class ADIS16488 : public ImuBase {
 public:
  ADIS16488(visensor::SensorId::SensorId sensor_id, int config_address,
      int bufferAddress, int bus_address);
  virtual ~ADIS16488();

  static bool checkPresence(int bus_address);
  virtual SharedRingBuffer* data_mover(){return &data_mover_;};

  virtual int getSensorType(){return visensor::SensorType::IMU_ADIS16488;};
  virtual int getSensorPort(){return port_number_;};
  virtual void on(uint32_t rate);
  virtual void off();

  virtual void writeUbiRegister(unsigned char reg_addr, int16_t data);
  virtual void readUbiRegister(unsigned char reg_addr, int16_t& data);

 private:
  UnifiedBusInterface bus_;
};

#endif /* ADIS16488_HPP_ */
