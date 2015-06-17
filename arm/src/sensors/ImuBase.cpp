/*
 * IMU.cpp
 *
 *  Created on: Aug 15, 2013
 *      Author: pascal
 */

#include "sensors/ImuBase.hpp"
ImuBase::ImuBase(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress,
                 const int data_size, const int length)
    : Sensor(sensor_id, ctrl_address, 12),
      data_mover_(data_size, length, bufferAddress){
}

ImuBase::~ImuBase() {
}
