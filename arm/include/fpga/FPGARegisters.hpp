/*
 * FPGAConfig.h
 *
 *  Created on: Jul 10, 2013
 *      Author: pascal
 */

#ifndef FPGACONFIG_H_
#define FPGACONFIG_H_

#include <stdio.h>
#include "fpga/DevMem.h"

// FPGA Registers:
#define FPGA_SOFTWARE_VERSION  0x0000  // a increment of 100 means new incompatible interface
// Hardware register start addresses
//#define CAM0_REGISTER_OFFSET   0X0010
//#define CAM1_REGISTER_OFFSET   0X0030
//#define CAM2_REGISTER_OFFSET   0X0050
//#define CAM3_REGISTER_OFFSET   0X0070

// ring buffer registers
#define CONTROL                0x0000   // | 0: ENABLE | 1:....
#define BUFFER_ADDRESS         0x0004
#define BUFFER_LENGTH          0x0008
#define BUFFER_FPGA_POINTER    0x000c
#define BUFFER_LINUX_POINTER   0x0010
#define BUFFER_PACKAGE_SIZE    0x0014

// camera specific registers
#define CAM_REGISTER_BYTES     32
#define CAM_TRIGGER_FREQUENCY  0x0014
#define CAM_RESERVED1          0x0018
#define CAM_RESERVED2          0x001C

// timing block registers
#define TIME_REGISTERS_BYTES   12
#define TIME_CTRL              0x0008
#define TIME_WR                0x0004
#define TIME_RD                0x0000

// control registers
#define R_FRAME         0x08
#define R_START_OFFSET  0x04
#define R_INTERVAL      0x00

// FLIR control registers
#define R_FLIR_START_OFFSET  0x04
#define R_FLIR_INTERVAL      0x00
#define R_FLIR_SERIALIZER_RESET 0x08

#define EX_TRIGGER_REGISTERS_BYTES  40
#define R_EX_TRIGGER_CONTROL      0x00 // 0=OFF, 1=ON
#define R_EX_TRIGGER_DIRECTION    0x04 // 0=out, 1=in
#define R_EX_TRIGGER_POLARITY     0x08 // 0=active high, 1=low active
#define R_EX_TRIGGER_LENGHT       0x0C // defines the impulse length in fpga zyklen (125MHz)
#define R_EX_TRIGGER0_START_OFFSET 0x10
#define R_EX_TRIGGER0_INTERVAL     0x14
#define R_EX_TRIGGER1_START_OFFSET 0x18
#define R_EX_TRIGGER1_INTERVAL     0x1C
#define R_EX_TRIGGER2_START_OFFSET 0x20
#define R_EX_TRIGGER2_INTERVAL     0x24

// light control registers
#define R_LIGHT_MODE 0X00
#define R_LIGHT_ONTIME 0X04

// zynq status (addresses two different modules: temperature and version register)
// temperature from AXI XADC (see http://www.xilinx.com/support/documentation/ip_documentation/axi_xadc/v1_00_a/pg019_axi_xadc.pdf)
#define R_ZSTATUS_BIT_VER_BASE		0x78440000	//Base address of bitstream version module
#define R_ZSTATUS_BIT_VER_MAJOR		0x00		//Bitstream version major number (32bit)
#define R_ZSTATUS_BIT_VER_MINOR		0x04		//Bitstream version minor number (32bit)
#define R_ZSTATUS_BIT_VER_PATCH		0x08		//Bitstream version patch number (32bit)
#define R_ZSTATUS_BIT_VER_NOTE		0x0C		//Bitstream note string (16 byte, 4x 32bit reg...)
#define R_ZSTATUS_IMU_CONNECTOR_CONFIG 0X10    //the LSB defines the pin mapping on the connector (0: new, 1: old PCB)

#define R_ZSTATUS_TEMP_BASE			0x78441000	//Base address of AXI XADC module
#define R_ZSTATUS_TEMP_CURRENT 		0X1200		//ADC value of current temperature
#define R_ZSTATUS_TEMP_MAX 			0X1280		//ADC value of max temperature since reset
#define R_ZSTATUS_TEMP_MIN 			0X1290		//ADC value of min temperature since reset



class FPGARegisters {
 public:
  FPGARegisters(long start_address, const int size /*in Bytes*/);
  virtual ~FPGARegisters();

  int getRegisterValue(off_t offset /*in Bytes*/) const;
  void setRegisterValue(off_t offset /*in Bytes*/, const int value);

  void print_register_values();

 private:
  DevMem mem;
  int size_;
};

#endif /* FPGACONFIG_H_ */
