/*
 * FPGAConfig.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: pascal
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "fpga/FPGARegisters.hpp"

FPGARegisters::FPGARegisters(long start_address, const int size)
: mem(start_address, size),
  size_(size){
}

FPGARegisters::~FPGARegisters() {
}

void FPGARegisters::print_register_values() {
  printf("==============================\n");
  printf("Offset  Hex         Int\n");
  for(int i = 0; i < size_; i+=4)
  {
    printf("0x%04X: 0x%08X  %d\n", i, getRegisterValue(i), getRegisterValue(i));
  }
  printf("==============================\n");
}

int FPGARegisters::getRegisterValue(off_t offset) const{
  return mem.read_word(offset);
}

void FPGARegisters::setRegisterValue(off_t offset, const int value) {
  mem.write_word(offset, value);
}
