/*
 * ADIS16448.cpp
 *
 *  Created on: Apr 2, 2014
 *      Author: pascal
 */

#include "sensors/ADIS16448.hpp"

#define ADIS_MAX_RATE 800
#define ADIS_MIN_RATE 1
#define FRAME_LENGHT 10
#define IMU_DATA_SIZE (6*2+5*2)*FRAME_LENGHT+4+4
#define IMU_BUFFER_LENGTH 10000

ADIS16448::ADIS16448(visensor::SensorId::SensorId sensor_id, int config_address,
                     int bufferAddress, int bus_address)
    : ImuBase(sensor_id, config_address, bufferAddress, IMU_DATA_SIZE, IMU_BUFFER_LENGTH),
      bus_(bus_address, 0x00){
}

ADIS16448::~ADIS16448() {
}


bool ADIS16448::checkPresence(int bus_address){
  //create bus interface
  UnifiedBusInterface bus(bus_address, 0x00);

  //check if imu is availbale by reading the ID
  bool imu_available=false;
  int16_t imu_id;

  for(int i = 0; i<1000 && !imu_available; i++)
  {
    bus.read16bRegSPI(0x56, imu_id);
    if(imu_id  == 0x4040)
      imu_available = true;
    else
      printf("ERROR: IMU ID not 0x4040!!! got: 0x%04X\n", imu_id);
  }

  if(imu_available)
    printf("Adis16448 detected\n");

  return imu_available;
}

void ADIS16448::on(uint32_t rate) {
  //read imu id
  int16_t imu_id;
  bus_.read16bRegSPI(0x56, imu_id);

  printf("==============================\n");
  printf("Read IMU ID: 0x%04X\n", imu_id);
  printf("R_START_OFFSET: %d\n", config_registers_.getRegisterValue(R_START_OFFSET));
  printf("R_INTERVAL: %d\n", config_registers_.getRegisterValue(R_INTERVAL));
  printf("==============================\n");

  // start receiving data
  data_mover_.on();

  //constrain data rate
  if(rate > ADIS_MAX_RATE) rate = ADIS_MAX_RATE;
  if(rate < ADIS_MIN_RATE) rate = ADIS_MIN_RATE;

  // configure time stuff
  config_registers_.setRegisterValue(R_INTERVAL, 100000/rate/*Hz*/);
  config_registers_.setRegisterValue(R_START_OFFSET, 100000/100/*s*/);
  config_registers_.setRegisterValue(R_FRAME, FRAME_LENGHT);

  active_ = true;
}

void ADIS16448::off() {
  //turn off the imu core
  config_registers_.setRegisterValue(R_INTERVAL, 0/*Hz*/);

  active_ = false;
  data_mover_.off();
}

void ADIS16448::writeUbiRegister(unsigned char reg_addr, int16_t data) {
  bus_.write16bReg(reg_addr, data); //TODO make spi
}

void ADIS16448::readUbiRegister(unsigned char reg_addr, int16_t& data) {
  bus_.read16bRegSPI(reg_addr, data);
}
