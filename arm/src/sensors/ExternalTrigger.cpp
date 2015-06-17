/*
 * ExternalTrigger.cpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#include "sensors/ExternalTrigger.hpp"

ExternalTrigger::ExternalTrigger(visensor::SensorId::SensorId sensor_id, int config_address, int bufferAddress)
    : Sensor(sensor_id, config_address, EX_TRIGGER_REGISTERS_BYTES),
      data_mover_(EXTERNAL_TRIGGER_DATA_SIZE, EXTERNAL_TRIGGER_BUFFER_LENGTH,
                  bufferAddress) {
  // make sure trigger is off, otherwise fpga can crash if hokuyo is turned on
  config_registers_.setRegisterValue(R_EX_TRIGGER_CONTROL, 0);

  //set default config
  setDefaultConfig();
}

ExternalTrigger::~ExternalTrigger() {
  off();
}

void ExternalTrigger::setDefaultConfig(void)
{
  //default config
  config_registers_.setRegisterValue(R_EX_TRIGGER_DIRECTION, 7); // 0b111 // 0=out, 1=in
  config_registers_.setRegisterValue(R_EX_TRIGGER_POLARITY, 0); // 000 // 0=active high, 1=low active
  config_registers_.setRegisterValue(R_EX_TRIGGER_LENGHT, 100000); // impulse length in fpga zyklen (125MHz)

  printf("==============================\n");
  printf("External Triggers Configured\n");
  printf("R_EX_TRIGGER_DIRECTION: %d\n", config_registers_.getRegisterValue(R_EX_TRIGGER_DIRECTION));
  printf("R_EX_TRIGGER_POLARITY: %d\n", config_registers_.getRegisterValue(R_EX_TRIGGER_POLARITY));
  printf("R_EX_TRIGGER_LENGHT: %d\n", config_registers_.getRegisterValue(R_EX_TRIGGER_LENGHT));
  printf("==============================\n");
}

void ExternalTrigger::on(uint32_t rate) {

  // start receiving data
  data_mover_.on();

  //constrain data rate
  if (rate > EXTERNAL_TRIGGER_MAX_RATE)
    rate = EXTERNAL_TRIGGER_MAX_RATE;
  if (rate < EXTERNAL_TRIGGER_MIN_RATE)
    rate = EXTERNAL_TRIGGER_MIN_RATE;

  // configure fpga block trigger0 (input laser)
  config_registers_.setRegisterValue(R_EX_TRIGGER0_START_OFFSET, 1000 /*s/100000*/);
  config_registers_.setRegisterValue(R_EX_TRIGGER0_INTERVAL, 100000 / rate/*Hz*/);

  printf("==============================\n");
  printf("External Trigger Started\n");
  printf("R_START_OFFSET: %d\n",
         config_registers_.getRegisterValue(R_EX_TRIGGER0_START_OFFSET));
  printf("R_INTERVAL: %d\n", config_registers_.getRegisterValue(R_EX_TRIGGER0_INTERVAL));
  printf("==============================\n");

  // configure fpga block trigger1 (input laser)

  config_registers_.setRegisterValue(R_EX_TRIGGER1_START_OFFSET, 2000 /*s/100000*/);
  config_registers_.setRegisterValue(R_EX_TRIGGER1_INTERVAL, 100000 / rate/*Hz*/);

  printf("==============================\n");
  printf("External Trigger Started\n");
  printf("R_START_OFFSET: %d\n",
         config_registers_.getRegisterValue(R_EX_TRIGGER1_START_OFFSET));
  printf("R_INTERVAL: %d\n", config_registers_.getRegisterValue(R_EX_TRIGGER1_INTERVAL));
  printf("==============================\n");

  //configure fpga block trigger2 (output GPS)
  config_registers_.setRegisterValue(R_EX_TRIGGER2_START_OFFSET, 3000 /*s/100000*/);
  config_registers_.setRegisterValue(R_EX_TRIGGER2_INTERVAL, 100000 / rate/*Hz*/);

  // Activate trigger 0
  config_registers_.setRegisterValue(R_EX_TRIGGER_CONTROL, 7); // 111 // 0=OFF, 1=ON

  printf("==============================\n");
  printf("External Triggers Configured\n");
  printf("R_EX_TRIGGER_CONTROL: %d\n", config_registers_.getRegisterValue(R_EX_TRIGGER_CONTROL));
  printf("==============================\n");

  active_ = true;
}

void ExternalTrigger::off() {
  config_registers_.setRegisterValue(R_EX_TRIGGER_CONTROL, 0);
  active_ = false;
  data_mover_.off();
}

