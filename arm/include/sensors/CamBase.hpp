/*
 * CamBase.hpp
 *
 *  Created on: Aug 14, 2013
 *      Author: pascal
 */

#ifndef CAM_BASE_HPP_
#define CAM_BASE_HPP_

#include "fpga/FPGARegisters.hpp"
#include "fpga/SharedRingBuffer.hpp"

#include "Sensor.hpp"
#include "visensor/visensor_constants.hpp"


class CamBase: public Sensor  {
 public:
  CamBase(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress, const int size,
          const int data_size, const int length);
  virtual ~CamBase();

  virtual SharedRingBuffer* data_mover(){return &data_mover_;};

  virtual int getCamBaseType(){return 0;};
  virtual void on(uint32_t rate){};
  virtual void off(){};

  virtual void writeUbiRegister(unsigned char reg_addr, int16_t data){};
  virtual void readUbiRegister(unsigned char reg_addr, int16_t& data){};

 protected:
  SharedRingBuffer data_mover_;
};

#endif /* CamBase_HPP_ */
