/*
 * MPU9150.cpp
 *
 *  Created on: Apr 2, 2014
 *      Author: pascal
 */

#include "sensors/MPU9150.hpp"

#define MPU_MAX_RATE 800
#define MPU_MIN_RATE 1
#define FRAME_LENGHT 10
#define MPU_DATA_SIZE (6*2+6*2)*FRAME_LENGHT+4+4
#define MPU_BUFFER_LENGTH 10000
#define MPU9150_I2C_ADDRESS 0x68

MPU9150::MPU9150(visensor::SensorId::SensorId sensor_id, int config_address,
                     int bufferAddress, int bus_address)
    : ImuBase(sensor_id, config_address, bufferAddress, MPU_DATA_SIZE, MPU_BUFFER_LENGTH),
      bus_(bus_address, MPU9150_I2C_ADDRESS){
}

MPU9150::~MPU9150() {
}

bool MPU9150::checkPresence(int bus_address){

  //create bus interface
  UnifiedBusInterface bus(bus_address, MPU9150_I2C_ADDRESS);

  //check if imu is availbale by reading the ID
  bool imu_available=false;
  int8_t imu_id;

  for(int i = 0; i<10 && !imu_available; i++)
  {
    bus.read8bReg(0x75, imu_id);
    if(imu_id  == 0x68)
      imu_available = true;
    else{
      printf("ERROR: IMU ID not 0x68!!! got: 0x%1X on bus at address: 0x%X\n", (unsigned)(unsigned char)imu_id, bus_address);
      usleep(1);
    }
  }

  if(imu_available)
    printf("MPU-9150 detected on bus at address: 0x%X\n", bus_address);

  return imu_available;
}

void MPU9150::on(uint32_t rate) {
  //read imu id
  int8_t imu_id;
  bus_.read8bReg(0x56, imu_id);

  printf("==============================\n");
  printf("Read IMU ID: 0x%04X\n", imu_id);
  printf("R_START_OFFSET: %d\n", config_registers_.getRegisterValue(R_START_OFFSET));
  printf("R_INTERVAL: %d\n", config_registers_.getRegisterValue(R_INTERVAL));
  printf("==============================\n");

  // start receiving data
  data_mover_.on();

  //constrain data rate
  if(rate > MPU_MAX_RATE) rate = MPU_MAX_RATE;
  if(rate < MPU_MIN_RATE) rate = MPU_MIN_RATE;

  // configure time stuff
  config_registers_.setRegisterValue(R_INTERVAL, 100000/rate/*Hz*/);
  config_registers_.setRegisterValue(R_START_OFFSET, 100000/100/*s*/);
  config_registers_.setRegisterValue(R_FRAME, FRAME_LENGHT);

  active_ = true;
}

void MPU9150::off() {
  //turn off the imu core
  config_registers_.setRegisterValue(R_INTERVAL, 0/*Hz*/);

  active_ = false;
  data_mover_.off();
}

void MPU9150::writeUbiRegister(unsigned char reg_addr, int16_t data) {
  bus_.write16bReg(reg_addr, data);
}

void MPU9150::readUbiRegister(unsigned char reg_addr, int16_t& data) {
  bus_.read16bReg(reg_addr, data);
}
