/*
 * FpgaConstants.hpp
 *
 *  Created on: Dec 18, 2013
 *      Author: schneith
 */

#ifndef FPGACONSTANTS_HPP_
#define FPGACONSTANTS_HPP_


//keep all these constants in sync with the FPGA code

namespace FPGA_CONSTANTS {

/* IRQ definitions */
namespace IRQ {
const int CAM0 = 88;
const int CAM1 = 87;
const int CAM2 = 91;
const int CAM3 = 90;
const int IMU0 = 89;
}

typedef const unsigned int address_t;

//Sensors
address_t IMU0_ADDRESS_BUS = 0x7e00000c;
address_t IMU0_ADDRESS_BUFFER = 0x7e360000;
address_t IMU0_ADDRESS_CONFIG = 0x7e000000;

address_t MPU0_ADDRESS_BUS = 0X8000000c;
address_t MPU0_ADDRESS_BUFFER = 0x80002000;
address_t MPU0_ADDRESS_CONFIG = 0X80000000;

address_t MPU1_ADDRESS_BUS = 0X8000100c;
address_t MPU1_ADDRESS_BUFFER = 0x80003000;
address_t MPU1_ADDRESS_CONFIG = 0X80001000;

address_t CAM0_ADDRESS_BUS = 0x7e040014;
address_t CAM0_ADDRESS_BUFFER = 0x7e200000;
address_t CAM0_ADDRESS_CONFIG = 0x7e040008;

address_t CAM1_ADDRESS_BUS = 0x7e080014;
address_t CAM1_ADDRESS_BUFFER = 0x7e240000;
address_t CAM1_ADDRESS_CONFIG = 0x7e080008;

address_t CAM2_ADDRESS_BUS = 0x7e0C0014;
address_t CAM2_ADDRESS_BUFFER = 0x7e280000;
address_t CAM2_ADDRESS_CONFIG = 0x7e0C0008;

address_t CAM3_ADDRESS_BUS = 0x7e100014;
address_t CAM3_ADDRESS_BUFFER = 0x7e2C0000;
address_t CAM3_ADDRESS_CONFIG = 0x7e100008;

address_t FLIR3_ADDRESS_BUS = 0x806000c;
address_t FLIR3_ADDRESS_BUFFER = 0x80B00000;
address_t FLIR3_ADDRESS_CONFIG = 0x80600000;

address_t CORNER0_ADDRESS_BUFFER = 0x80800000;
address_t CORNER0_ADDRESS_CONFIG = 0x80400000;

address_t CORNER1_ADDRESS_BUFFER = 0x80840000;
address_t CORNER1_ADDRESS_CONFIG = 0x80440000;

address_t DENSE_ADDRESS_BUFFER = 0x7e280000; // HACK:(gohlp) address from cam2
address_t DENSE_ADDRESS_CONFIG = 0x7f000000;

address_t EX_TRIGGER_BUFFER = 0x7e400000;
address_t EX_TRIGGER_CONFIG = 0x7e640000;

address_t LIGHT_CONTROL_REGCONF = 0x7e040000;

address_t TIMING_BLOCK_REGCONF = 0x78400000;

}


#endif /* FPGACONSTANTS_HPP_ */
