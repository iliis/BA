/*
 * Sensor.cpp
 *
 *  Created on: Aug 14, 2013
 *      Author: pascal
 */

#include "sensors/Sensor.hpp"

//initialize sensor id
size_t Sensor::next_stream_ = 0;

Sensor::Sensor(visensor::SensorId::SensorId sensor_id, long config_registers_address, const int size)
    : config_registers_(config_registers_address, size),
      active_(false),
      sensor_id_(sensor_id)
{
}

Sensor::~Sensor() {
}

void Sensor::writeFpgaRegister(unsigned char reg_addr, int32_t data) {
  config_registers_.setRegisterValue(reg_addr, data);
}

void Sensor::readFpgaRegister(unsigned char reg_addr, int32_t& data) {
  data = config_registers_.getRegisterValue(reg_addr);
}
