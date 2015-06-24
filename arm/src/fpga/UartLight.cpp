#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <iostream>
#include <cstdlib>
#include <string.h>

#include "fpga/UartLight.hpp"

// Control bits
#define RESET_TX 1<<0
#define RESET_RX 1<<1

// Status bits
#define OUTFIFO_FULL  (1<<3)
#define INFIFO_FULL   (1<<1)
#define INFIFO_DATA   (1<<0)

// register addresses
#define R_CONTROL 0x0c
#define R_STATUS 0x08
#define R_DOUT 0x4
#define R_DIN 0x00

UartLight::UartLight(int register_address) :
mem(register_address, 4*4) {
  // clear RX buffer
  mem.write_word(R_CONTROL, RESET_RX);
}

UartLight::~UartLight() {
}

void UartLight::waitForNotFullOutBuffer(){

  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 1000;

  // check that the output buffer is not full
  for(int i = 0; (mem.read_word(R_STATUS) & OUTFIFO_FULL) && i < 20; i++)
  {
    if(i>19)
    //  printf("waiting for OUTFIFO_FULL not full: %04x\n", mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }

}

void UartLight::waitForNotFullInBuffer() {

  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 1000;

  // check that the input buffer is not full
  for(int i = 0; (mem.read_word(R_STATUS) & INFIFO_FULL) && i < 20; i++)
  {
    if(i>19)
    //  printf("waiting for INFIFO_FULL not full.\n");
    nanosleep(&slptm,NULL);
  }
}

void UartLight::waitForNotEmptyInBuffer() {

  struct timespec slptm;
  slptm.tv_sec = 0;
  slptm.tv_nsec = 1000;

  for(int i = 0; !(mem.read_word(R_STATUS) & INFIFO_DATA) && i < 100; i++)
  {
    // printf("waiting for INFIFO_EMPTY not empty: %04x.\n", mem.read_word(R_STATUS));
    nanosleep(&slptm,NULL);
  }
}

/********************************************************************
 *This function writes one byte of data "data" on the bus
 ********************************************************************/
int UartLight::writeByte(int8_t data) {

  waitForNotFullOutBuffer();

  // write data
  mem.write_word(R_DOUT, data);
  return 0;
}

/********************************************************************
 *This function reads one byte of data "data" from the bus
******************************************/
int UartLight::readByte(int8_t &data) {

  waitForNotEmptyInBuffer();

  // printf("read_word status: %04x.\n", mem.read_word(R_STATUS));
  data = mem.read_word(R_DIN);
  return 0;
}

