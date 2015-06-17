/*
 * MPU9150.hpp
 *
 *  Created on: Apr 2, 2014
 *      Author: pascal
 */

#ifndef MPU9150_HPP_
#define MPU9150_HPP_

#include "ImuBase.hpp"

class MPU9150 : public ImuBase {
 public:
  MPU9150(visensor::SensorId::SensorId sensor_id, int config_address,
      int bufferAddress, int bus_address);
  virtual ~MPU9150();

  static bool checkPresence(int bus_address);
  virtual SharedRingBuffer* data_mover(){return &data_mover_;};

  virtual int getSensorType(){return visensor::SensorType::MPU_9150;};
  virtual int getSensorPort(){return port_number_;};
  virtual void on(uint32_t rate);
  virtual void off();

  virtual void writeUbiRegister(unsigned char reg_addr, int16_t data);
  virtual void readUbiRegister(unsigned char reg_addr, int16_t& data);

 private:
  UnifiedBusInterface bus_;
};

#endif /* MPU9150_HPP_ */
