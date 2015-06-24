#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <cstdlib>
#include <string.h>

#include "fpga/UnifiedBusInterface.hpp"

// Bit definitions
#define READ 1<<0
#define WRITE 0<<0
#define START 1<<1
#define STOP 1<<2
#define ACK 1<<3
#define AXI_CTRL 1<<7

#define OUTFIFO_EMPTY  (1<<3)
#define OUTFIFO_FULL  (1<<2)
#define INFIFO_EMPTY  (1<<1)
#define INFIFO_FULL  (1<<0)

// register addresses
#define R_CONTROL 0x0c
#define R_STATUS 0x08
#define R_DOUT 0x4
#define R_DIN 0x00

//#define R_CONTROL 0x00
//#define R_STATUS 0x04
//#define R_DOUT 0x08
//#define R_DIN 0x0c
//#define R_FRAME 0x10

// from: http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.faqs/ka10391.html
#define LOWBYTE(v)   ((unsigned char) (v))
#define HIGHBYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))

UnifiedBusInterface::UnifiedBusInterface(int register_address, unsigned char device_address) :
mem(register_address, 16),
deviceAddress(device_address) {
}


UnifiedBusInterface::~UnifiedBusInterface() {
}

//void UnifiedBusInterface::addressSet(unsigned char address)
//{
//	deviceAddress = address;
//}


void UnifiedBusInterface::waitForNotFullOutBuffer(){

  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 1000;

  // check that the output buffer is not full
  for(int i = 0; (mem.read_word(R_STATUS) & OUTFIFO_FULL) && i < 20; i++)
  {
    if(i>19)
      printf("waiting for OUTFIFO_FULL not full: %04x\n", mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }

}

void UnifiedBusInterface::waitForNotFullInBuffer() {

  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 1000;

  // check that the input buffer is not full
  for(int i = 0; (mem.read_word(R_STATUS) & INFIFO_FULL) && i < 20; i++)
  {
    if(i>19)
      printf("waiting for INFIFO_FULL not full.\n");
    nanosleep(&slptm,NULL);
  }
}

void UnifiedBusInterface::waitForNotEmptyInBuffer() {

  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 1000;

  for(int i = 0; (mem.read_word(R_STATUS) & INFIFO_EMPTY) && i < 10; i++)
  {
    // printf("waiting for INFIFO_EMPTY not empty: %04x.\n", mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }
}

/********************************************************************
 *This function writes one byte of data "data" to a specific register
 *"reg_addr" in the I2C device
 ********************************************************************/
int UnifiedBusInterface::write8bReg(unsigned char reg_addr, int8_t data) {

  waitForNotFullOutBuffer();

  // set to write and start
  mem.write_word(R_CONTROL, WRITE | START | ACK);
  //write slave address with write bit
  mem.write_word(R_DOUT, deviceAddress << 1);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | ACK);
  // write register address
  mem.write_word(R_DOUT, reg_addr);

  waitForNotFullOutBuffer();

  // set to write
  mem.write_word(R_CONTROL, WRITE | STOP | ACK);
  // write data
  mem.write_word(R_DOUT, data);
  return 0;
}

/********************************************************************
 *This function reads one byte of data "data" from a specific register
 *"reg_addr" in the bus device. This involves sending the register address
 *byte "reg_Addr" with "write" asserted and then instructing the
 *I2C device to read a byte of data from that address ("read asserted").
******************************************/
int UnifiedBusInterface::read8bReg(unsigned char reg_addr, int8_t &data) {

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();

  // HACK to be sure
  // read input fifo until it is empty
  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 10000;

  for(int i = 0; (mem.read_word(R_STATUS) & (1 << 1)) == 0 && i < 16; i++)
  {
    printf("empty input buffer: %04x status: %04x\n", mem.read_word(R_DIN), mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }

  waitForNotFullOutBuffer();
  // set to write and start
  mem.write_word(R_CONTROL, WRITE | START | ACK | AXI_CTRL);
  //write slave address with write bit
  mem.write_word(R_DOUT, deviceAddress << 1);
//  printf("deviceAddress: %04x\n", deviceAddress << 1);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | ACK | AXI_CTRL);
  // write register address
  mem.write_word(R_DOUT, reg_addr);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE |  START |  ACK | AXI_CTRL);
  //write slave address with read bit
  mem.write_word(R_DOUT, deviceAddress << 1 | 0x1);

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();
  // set to read
  mem.write_word(R_CONTROL, READ | ACK | AXI_CTRL);
  // trigger reading of one byte
  mem.write_word(R_DOUT, 0x00);

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();
  // set to read
  mem.write_word(R_CONTROL, READ | STOP);
  // trigger reading of one byte
  mem.write_word(R_DOUT, 0x00);

  waitForNotEmptyInBuffer();
  // read lower data
  data = mem.read_word(R_DIN);

  return 0;
}

/********************************************************************
 *This function writes two bytes of data "data" to a specific register
 *"reg_addr" in the I2C device
 ********************************************************************/
int UnifiedBusInterface::write16bReg(unsigned char reg_addr, int16_t data) {
	//	if(mem.read_word(0x104) & (1 << 6))
	//		printf("rx empty\n");
	//	if(mem.read_word(0x104) & (1 << 7))
	//		printf("tx empty\n");

  waitForNotFullOutBuffer();

	// set to write and start
  mem.write_word(R_CONTROL, WRITE | START | ACK);
	//write slave address with write bit
	mem.write_word(R_DOUT, deviceAddress << 1);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | ACK);
	// write register address
  mem.write_word(R_DOUT, reg_addr);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | ACK);
	// write lower data
	mem.write_word(R_DOUT, HIGHBYTE(data));

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | STOP | ACK);
	// write upper data
	mem.write_word(R_DOUT, LOWBYTE(data));
	return 0;
}

/********************************************************************
 *This function reads two bytes of data "data" from a specific register
 *"reg_addr" in the bus device. This involves sending the register address
 *byte "reg_Addr" with "write" asserted and then instructing the
 *I2C device to read a byte of data from that address ("read asserted").
******************************************/
int UnifiedBusInterface::read16bReg(unsigned char reg_addr, int16_t &data) {

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();

  // HACK to be sure
  // read input fifo until it is empty
  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 10000;

  for(int i = 0; (mem.read_word(R_STATUS) & (1 << 1)) == 0 && i < 16; i++)
  {
    printf("empty input buffer: %04x status: %04x\n", mem.read_word(R_DIN), mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }

  waitForNotFullOutBuffer();
  // set to write and start
  mem.write_word(R_CONTROL, WRITE | START | ACK | AXI_CTRL);
  //write slave address with write bit
  mem.write_word(R_DOUT, deviceAddress << 1);
//  printf("deviceAddress: %04x\n", deviceAddress << 1);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | ACK | AXI_CTRL);
  // write register address
  mem.write_word(R_DOUT, reg_addr);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE |  START |  ACK | AXI_CTRL);
  //write slave address with read bit
  mem.write_word(R_DOUT, deviceAddress << 1 | 0x1);

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();
  // set to read
  mem.write_word(R_CONTROL, READ | ACK | AXI_CTRL);
  // trigger reading of one byte
  mem.write_word(R_DOUT, 0x00);

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();
  // set to read
  mem.write_word(R_CONTROL, READ | STOP);
  // trigger reading of one byte
  mem.write_word(R_DOUT, 0x00);

  waitForNotEmptyInBuffer();
  // read lower data
  int hi = mem.read_word(R_DIN);
  // set to write

  waitForNotEmptyInBuffer();
  // read upper data
  int lo = mem.read_word(R_DIN);

  // concatenate bytes
	data = lo | (hi<<8);

	return 0;
}
/********************************************************************
 *This function reads two bytes of data "data" from a specific register
 *"reg_addr" in the bus device. This involves sending the register address
 *byte "reg_Addr" with "write" asserted and then instructing the
 *I2C device to read a byte of data from that address ("read asserted").
******************************************/
int UnifiedBusInterface::read16bRegSPI(unsigned char reg_addr, int16_t &data) {

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();

  // HACK to be sure
  // read input fifo until it is empty
  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 10000;

  for(int i = 0; (mem.read_word(R_STATUS) & (1 << 1)) == 0 && i < 16; i++)
  {
    printf("empty input buffer: %04x status: %04x\n", mem.read_word(R_DIN), mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }

  waitForNotFullOutBuffer();
  // set to write and start
  mem.write_word(R_CONTROL, WRITE | START | ACK | AXI_CTRL);
  //write slave address with write bit
  mem.write_word(R_DOUT, deviceAddress << 1);
//  printf("deviceAddress: %04x\n", deviceAddress << 1);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | ACK | AXI_CTRL);
  // write register address
  mem.write_word(R_DOUT, reg_addr);

  waitForNotFullOutBuffer();
  // set to write
  mem.write_word(R_CONTROL, WRITE | STOP |ACK | AXI_CTRL);
  // write register address
  mem.write_word(R_DOUT, 0x00);

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();
  // set to read
  mem.write_word(R_CONTROL, READ | ACK | AXI_CTRL);
  // trigger reading of one byte
  mem.write_word(R_DOUT, 0x00);

  waitForNotFullOutBuffer();
  waitForNotFullInBuffer();
  // set to read
  mem.write_word(R_CONTROL, READ | ACK  | STOP);
  // trigger reading of one byte
  mem.write_word(R_DOUT, 0x00);

  waitForNotEmptyInBuffer();
  // read lower data
  // mem.write_word(R_CONTROL, READ | ACK  | AXI_CTRL);
  int hi = mem.read_word(R_DIN);
  // set to write

  waitForNotEmptyInBuffer();
  // read upper data
  // mem.write_word(R_CONTROL, READ | ACK | STOP);
  int lo = mem.read_word(R_DIN);

  // concatenate bytes
  data = lo | (hi<<8);

  return 0;
}
