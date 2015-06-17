/*
 * Tau640.cpp
 *
 *  Created on: Jul 15013
 *      Author: pascal
 */

#include "sensors/Tau640.hpp"
#include <cstdlib>
#include <stdio.h>

#define TAU_IMAGE_SIZE 640*512*2+4+4 //*2 for 16bit
#define TAU_BUFFER_LENGTH 3

Tau640::Tau640(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress, int bus_address)
: CamBase(sensor_id, ctrl_address, bufferAddress, CAM_REGISTER_BYTES, TAU_IMAGE_SIZE, TAU_BUFFER_LENGTH),
  bus_(bus_address) {
  off();

  CamStatus status;

//  // send factory reset command
//  status = sendSetMessage(0x03, 0, 0);
//  if(status != CamStatus::CAM_OK)
//    printf("Tau responded on function %X with status: %X\n", 0x03, status);

  // send soft reset command
  // it seems like the tau needs some time after this command to accept new ones.
  status = sendSetMessage(0x02, 0, 0);
  if(status != CamStatus::CAM_OK)
    printf("Tau responded on function %X with status: %X\n", 0x02, status);
}

Tau640::~Tau640() {
  off();
}

bool Tau640::checkPresence(int bus_address){
  UartLight bus(bus_address);
  int8_t data;
  bus.readByte(data);
  if(data == 0x1324)
  {
    printf("Tau640 Camera detected\n");
    return true;
  }
  return false;
}


#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))

// implementation from http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gaca726c22a1900f9bad52594c8846115f
uint16_t crc_ccitt_update(uint16_t crc, uint8_t data) {
  int i;

  crc = crc ^ ((uint16_t)data << 8);
  for (i=0; i<8; i++)
  {
      if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
      else
          crc <<= 1;
  }

  return crc;
}

Tau640::CamStatus Tau640::sendSetMessage(int8_t function, int bytes, int8_t data[]) {

  CamStatus status = CamStatus::UNDEFINED;

  uint16_t crc = 0;
  bus_.writeByte(0x6e);
  crc = crc_ccitt_update(crc, 0x6e);
  bus_.writeByte(0x00);
  crc = crc_ccitt_update(crc, 0x00);
  bus_.writeByte(0x00);
  crc = crc_ccitt_update(crc, 0x00);
  bus_.writeByte(function);
  crc = crc_ccitt_update(crc, function);
  bus_.writeByte(0x00);
  crc = crc_ccitt_update(crc, 0x00);
  bus_.writeByte(bytes);
  crc = crc_ccitt_update(crc, bytes);

// CRC
  uint16_t crc1 = crc;
  bus_.writeByte(HIGHBYTE(crc1) );
  bus_.writeByte(LOWBYTE(crc1) );
  crc = crc_ccitt_update(crc, HIGHBYTE(crc1) );
  crc = crc_ccitt_update(crc, LOWBYTE(crc1) );

  for (int i = 0; i < bytes; i++) {
    bus_.writeByte(data[i]);
    crc = crc_ccitt_update(crc, data[i]);
  }

// CRC
  bus_.writeByte(HIGHBYTE(crc) );
  bus_.writeByte(LOWBYTE(crc) );

  int8_t data_in;
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  switch(data_in) {
    case 0x00: {
      status = CamStatus::CAM_OK;
      break;
    }
    case 0x01: {
      status = CamStatus::CAM_BUSY;
      break;
    }
    case 0x02: {
      status = CamStatus::CAM_NOT_READY;
      break;
    }
    case 0x03: {
      status = CamStatus::CAM_RANGE_ERROR;
      break;
    }
    case 0x04: {
      status = CamStatus::CAM_CHECKSUM_ERROR;
      break;
    }
    case 0x05: {
      status = CamStatus::CAM_UNDEFINED_PROCESS_ERROR;
      break;
    }
    case 0x06: {
      status = CamStatus::CAM_UNDEFINED_FUNCTION_ERROR;
      break;
    }
    case 0x07: {
      status = CamStatus::CAM_TIMEOUT_ERROR;
      break;
    }
    case 0x09: {
      status = CamStatus::CAM_BYTE_COUNT_ERROR;
      break;
    }
    case 0x0A: {
      status = CamStatus::CAM_FEATURE_NOT_ENABLED;
      break;
    }
  }
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  for (int i = 0; i < bytes; i++) {
    bus_.readByte(data_in);
   }
  bus_.readByte(data_in);
  bus_.readByte(data_in);

  return status;
}

Tau640::CamStatus Tau640::sendGetMessage(int8_t function, int bytes, int8_t data[]) {

  CamStatus status = CamStatus::UNDEFINED;

  uint16_t crc = 0;
  bus_.writeByte(0x6e);
  crc = crc_ccitt_update(crc, 0x6e);
  bus_.writeByte(0x00);
  crc = crc_ccitt_update(crc, 0x00);
  bus_.writeByte(0x00);
  crc = crc_ccitt_update(crc, 0x00);
  bus_.writeByte(function);
  crc = crc_ccitt_update(crc, function);
  bus_.writeByte(0x00);
  crc = crc_ccitt_update(crc, 0x00);
  bus_.writeByte(0);
  crc = crc_ccitt_update(crc, 0);

// CRC
  uint16_t crc1 = crc;
  bus_.writeByte(HIGHBYTE(crc1) );
  bus_.writeByte(LOWBYTE(crc1) );
  crc = crc_ccitt_update(crc, HIGHBYTE(crc1) );
  crc = crc_ccitt_update(crc, LOWBYTE(crc1) );

// CRC
  bus_.writeByte(HIGHBYTE(crc) );
  bus_.writeByte(LOWBYTE(crc) );

  int8_t data_in;
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  switch(data_in) {
    case 0x00: {
      status = CamStatus::CAM_OK;
      break;
    }
    case 0x01: {
      status = CamStatus::CAM_BUSY;
      break;
    }
    case 0x02: {
      status = CamStatus::CAM_NOT_READY;
      break;
    }
    case 0x03: {
      status = CamStatus::CAM_RANGE_ERROR;
      break;
    }
    case 0x04: {
      status = CamStatus::CAM_CHECKSUM_ERROR;
      break;
    }
    case 0x05: {
      status = CamStatus::CAM_UNDEFINED_PROCESS_ERROR;
      break;
    }
    case 0x06: {
      status = CamStatus::CAM_UNDEFINED_FUNCTION_ERROR;
      break;
    }
    case 0x07: {
      status = CamStatus::CAM_TIMEOUT_ERROR;
      break;
    }
    case 0x09: {
      status = CamStatus::CAM_BYTE_COUNT_ERROR;
      break;
    }
    case 0x0A: {
      status = CamStatus::CAM_FEATURE_NOT_ENABLED;
      break;
    }
  }
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  bus_.readByte(data_in);
  for (int i = 0; i < bytes; i++) {
    bus_.readByte(data_in);
   }
  bus_.readByte(data_in);
  bus_.readByte(data_in);

  return status;
}

void Tau640::turnTestPatternOn() {
  // trigger off
  int8_t data[2];
  CamStatus status;
  data[0] = 0;
  data[1] = 0;
  status = sendSetMessage(0x21, 2, data);
  if(status != CamStatus::CAM_OK)
    printf("Tau responded on function %X with status: %X\n", 0x21, status);

  // ffc manual
  data[0] = 0;
  data[1] = 0;
  status = sendSetMessage(0x0B, 2, data);
  if(status != CamStatus::CAM_OK)
    printf("Tau responded on function %X with status: %X\n", 0x0B, status);

  // gain manual
  data[0] = 0;
  data[1] = 3;
  status = sendSetMessage(0x0A, 2, data);
  if(status != CamStatus::CAM_OK)
    printf("Tau responded on function %X with status: %X\n", 0x0A, status);

  // test pattern
  data[0] = 0;
  data[1] = 1;
  status = sendSetMessage(0x25, 2, data);
  if(status != CamStatus::CAM_OK)
    printf("Tau responded on function %X with status: %X\n", 0x25, status);
}

void Tau640::configureFlir() {
  // set external trigger
  int8_t data[2];
  data[0] = 0;
  data[1] = 1;
  CamStatus status = sendSetMessage(0x21, 2, data);
  if (status != CamStatus::CAM_OK)
    printf("Tau responded on function %X with status: %X\n", 0x21, status);
}

void Tau640::on(uint32_t rate) {

  configureFlir();

  // reset fpga serializer
  config_registers_.setRegisterValue(R_FLIR_SERIALIZER_RESET, 1);

  data_mover_.on();

  //constrain data rate
  if(rate > TAU640_MAX_RATE) rate = TAU640_MAX_RATE;
  if(rate < TAU640_MIN_RATE) rate = TAU640_MIN_RATE;

  // configure time stuff
  config_registers_.setRegisterValue(R_FLIR_INTERVAL, 100000/rate/*Hz*/);
  config_registers_.setRegisterValue(R_FLIR_START_OFFSET, 100000/100/*s*/);

  active_ = true;
  printf("Tau cam turned on\n");
}

void Tau640::off() {
  //turn off the tau core
  config_registers_.setRegisterValue(R_FLIR_INTERVAL, 0/*Hz*/);

  active_=false;
  data_mover_.off();
}

void Tau640::writeUbiRegister(unsigned char reg_addr, int16_t data) {
//  bus_.write16bReg(reg_addr, data);
}

void Tau640::readUbiRegister(unsigned char reg_addr, int16_t& data) {
//  bus_.read16bReg(reg_addr, data);
}
