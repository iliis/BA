/*
 * LightControl.cpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#include <time.h>

#include "sensors/LightControl.hpp"

LightControl::LightControl(visensor::SensorId::SensorId sensor_id, int register_address)
: Sensor(sensor_id, register_address, 8){
}

LightControl::~LightControl() {
}

void LightControl::on() {

  // choose constant mode as default
  config_registers_.setRegisterValue(R_LIGHT_MODE, 0X01);

  // set to zero to turn the LED's off
  config_registers_.setRegisterValue(R_LIGHT_ONTIME, 0x00);

  // Debug stuff
  printf("========================================\n");
  printf("LightControl registers:\n");
  config_registers_.print_register_values();
  printf("========================================\n");
  active_=true;
}

void LightControl::off() {
  config_registers_.setRegisterValue(R_LIGHT_ONTIME, 0X00);
  active_=false;
}

void LightControl::setIntensity(int32_t value) {
  config_registers_.setRegisterValue(R_LIGHT_ONTIME, value);
}

int32_t LightControl::getIntensity() {
  return config_registers_.getRegisterValue(R_LIGHT_ONTIME);
}
