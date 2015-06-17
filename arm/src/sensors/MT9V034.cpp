/*
 * MT9V034.cpp
 *
 *  Created on: Jul 15013
 *      Author: pascal
 */

#include "sensors/MT9V034.hpp"

#include <cstdlib>
#include <stdio.h>

#define IMAGE_SIZE 752*480+4+4
#define IMAGE_BUFFER_LENGTH 8
#define I2C_ADDRESS 0x48

MT9V034::MT9V034(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress,
                 int bus_address)
    : CamBase(sensor_id, ctrl_address, bufferAddress, CAM_REGISTER_BYTES,
      IMAGE_SIZE, IMAGE_BUFFER_LENGTH),
      bus_(bus_address, I2C_ADDRESS) {
  printf("Camera created with sensor id: %d\n", id());
  off();
}

MT9V034::~MT9V034() {
  off();
}

bool MT9V034::checkPresence(int bus_address){
  UnifiedBusInterface bus(bus_address, I2C_ADDRESS);
  int16_t data;
  bus.read16bReg(0x00, data);
  if(data == 0x1324)
  {
    printf("MT9V034 Camera detected\n");
    return true;
  }
  return false;
}

void MT9V034::on(uint32_t rate) {

  printf("==============================\n");
  int16_t data;
  bus_.read16bReg(0x00, data);
  if(data != 0x1324)
  {
	  printf("ERROR: Camera ID not 0x1324!!! got: 0x%04X\n", data);

	  //do not initialize the sensor if the identification has failed
	  return;
  }

  bus_.write16bReg(0x07,0x0118);    //enable lvds clock
  bus_.write16bReg(0xB2,0x0000);    //enable lvds clock
  bus_.write16bReg(0xB6,0x0001);    //use 10 bit per pixel
  bus_.write16bReg(0xB1,0x0000);    //lvds control (not strictly necessary)
  bus_.write16bReg(0x20,0x03C7);    // dangerous reserved register :) more appropriate values according to "TN-09-225: MT9V024 Snapshot Exposure Mode Operation"
  //  i2c_bus.writeReg(0x7F, 1 << 12 | 1 << 13); // set test pattern
  //	i2c_bus.writeReg(0x7F, 0 << 12 | 0 << 13); // unset test pattern
  bus_.write16bReg(0x0C,0x0001);    //soft reset

  printf("Check camera register values:\n");
  bus_.read16bReg(0xB2, data);
//  printf("Register 0xB2 read value: 0x%04X\n", data);
  bus_.read16bReg(0xB6, data);
  printf("Register 0xB6 read value: 0x%04X\n", data);
  bus_.read16bReg(0xB1, data);
  printf("Register 0xB1 read value: 0x%04X\n", data);
  bus_.read16bReg(0x20, data);
  printf("Register 0x20 read value: 0x%04X\n", data);
  bus_.read16bReg(0x7F, data);
  printf("Register 0x7F read value: 0x%04X\n", data);
  //	i2c_bus.writeReg(0xB2,0x0000);    //enable lvds clock
  printf("==============================\n");

  data_mover_.on();

  //constrain data rate
  if(rate > MT9V034_MAX_RATE) rate = MT9V034_MAX_RATE;
  if(rate < MT9V034_MIN_RATE) rate = MT9V034_MIN_RATE;

  // configure time stuff
  config_registers_.setRegisterValue(R_INTERVAL, 100000/rate/*Hz*/);
  config_registers_.setRegisterValue(R_START_OFFSET, 100000/100/*s*/);
  config_registers_.setRegisterValue(R_FRAME, 1 );

  active_ = true;
}

void MT9V034::off() {
  //turn off the camera core
  config_registers_.setRegisterValue(R_INTERVAL, 0/*Hz*/);

  active_ = false;
  data_mover_.off();
}

void MT9V034::writeUbiRegister(unsigned char reg_addr, int16_t data) {
  bus_.write16bReg(reg_addr, data);
}

void MT9V034::readUbiRegister(unsigned char reg_addr, int16_t& data) {
  bus_.read16bReg(reg_addr, data);
}
