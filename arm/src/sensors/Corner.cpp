/*
 * Corners.cpp
 *
 *  Created on: Jul 15013
 *      Author: pascal
 */

#include "sensors/Corner.hpp"

#include <cstdlib>
#include <stdio.h>

#define CORNER_DATA_SIZE 1000*8
#define RING_BUFFER_LENGTH 9

Corner::Corner(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress)
    : CamBase(sensor_id, ctrl_address, bufferAddress, CAM_REGISTER_BYTES,
              CORNER_DATA_SIZE, RING_BUFFER_LENGTH) {
  printf("Corner created with sensor id: %d\n", id());
  off();
}

Corner::~Corner() {
  off();
}

void Corner::on(uint32_t rate) {
  data_mover_.on();
  config_registers_.setRegisterValue(0, CORNER_DATA_SIZE);
  config_registers_.setRegisterValue(4, 1<<24);
  active_ = true;
}

void Corner::off() {
  active_ = false;
  data_mover_.off();
}
