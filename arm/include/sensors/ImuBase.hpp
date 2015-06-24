/*
 * IMU.hpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include "fpga/UnifiedBusInterface.hpp"
#include "Sensor.hpp"

class ImuBase : public Sensor {
 public:
  ImuBase(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress,
      const int data_size, const int length);
  virtual ~ImuBase();

  static bool checkPresence(int bus_address);
  virtual SharedRingBuffer* data_mover(){return &data_mover_;};

  virtual int getSensorType() = 0;
  int getSensorPort(){return port_number_;};
  virtual void on(uint32_t rate) = 0;
  virtual void off() = 0;

  virtual void writeUbiRegister(unsigned char reg_addr, int16_t data) = 0;
  virtual void readUbiRegister(unsigned char reg_addr, int16_t& data) = 0;

protected:
  SharedRingBuffer data_mover_;
  int port_number_;
};

#endif /* IMU_HPP_ */
