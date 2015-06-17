/*
 * Sensor.hpp
 *
 *  Created on: Aug 14, 2013
 *      Author: pascal
 */

#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <fpga/FPGARegisters.hpp>
#include <fpga/SharedRingBuffer.hpp>

#include "visensor/visensor_constants.hpp"

class Sensor {
 public:
  typedef boost::shared_ptr<Sensor> Ptr;
  typedef std::map<visensor::SensorId::SensorId, Sensor::Ptr > Map;

  Sensor(visensor::SensorId::SensorId sensor_id, long config_registers_address, const int size);
  virtual ~Sensor();

  virtual SharedRingBuffer* data_mover(){return 0;};

  virtual int getSensorType(){return 0;};
  int id(){return sensor_id_;};
  virtual void on(uint32_t rate){};
  virtual void off(){};

  virtual void writeUbiRegister(unsigned char reg_addr, int16_t data){};
  virtual void readUbiRegister(unsigned char reg_addr, int16_t& data){};

  virtual void writeFpgaRegister(unsigned char reg_addr, int32_t data);
  virtual void readFpgaRegister(unsigned char reg_addr, int32_t& data);

  bool isActive() const {
    return active_;
  }

 protected:
  FPGARegisters config_registers_;
  volatile bool active_;

 private:
  const visensor::SensorId::SensorId sensor_id_;
  //unique stream counter
  static size_t next_stream_;
};

#endif /* SENSOR_HPP_ */
