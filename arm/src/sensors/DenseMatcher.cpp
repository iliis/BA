/*
 * DenseMatcher.cpp
 *
 *  Created on: Jul 15013
 *      Author: pascal
 */

#include "sensors/DenseMatcher.hpp"

#include <cstdlib>
#include <stdio.h>

#define DENSE_IMAGE_SIZE 752*480+4+4
#define DENSE_BUFFER_LENGTH 5
#define DENSE_CONTROL_BYTES 44*4
#define I2C_ADDRESS 0x48

DenseMatcher::DenseMatcher(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress)
    : CamBase(sensor_id, ctrl_address, bufferAddress, DENSE_CONTROL_BYTES,
              DENSE_IMAGE_SIZE, DENSE_BUFFER_LENGTH) {
  printf("Dense created with sensor id: %d\n", id());
  off();
}

DenseMatcher::~DenseMatcher() {
  off();
}

bool DenseMatcher::checkPresence(int bus_address){
  return true;
}

void DenseMatcher::on(uint32_t rate) {
  data_mover_.on();

  printf("DenseMatcher started\n");
  active_ = true;
}

void DenseMatcher::off() {
  active_ = false;
  data_mover_.off();
}
