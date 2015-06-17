/*
 * CamBase.cpp
 *
 *  Created on: Aug 14, 2013
 *      Author: pascal
 */

#include "sensors/CamBase.hpp"

//CamBase::CamBase()
//: registers(0x7e200000, 16),
//  buffer_(IMAGE_SIZE, 8){
////  registers.setRegisterValue(CAM_BUFFER_ADDRESS, buffer_.get_start_address());
////  registers.setRegisterValue(CAM_BUFFER_SIZE, IMAGE_SIZE*8);
////  registers.setRegisterValue(CAM_CONTROL, 0x01);
////  registers.print_register_values();
//}

CamBase::CamBase(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress,
                 const int size, const int data_size, const int length)
    : Sensor(sensor_id, ctrl_address, size),
      data_mover_(data_size, length, bufferAddress) {
}

CamBase::~CamBase() {

}
