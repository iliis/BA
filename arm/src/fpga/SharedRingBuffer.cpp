/*
 * shared_memory.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pascal
 */
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/foreach.hpp>
#include <stdint.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

#include "fpga/SharedRingBuffer.hpp"

///////////////////////////////////////////////////////////////////////////////////////////////
//
// fpga pointer: points to last element that has been written successfully from FPGA
//                --> fpga pointer is written and keept at fpga register
//
// linux pointer: points to last element that has been read successfully from ARM
//                --> linux pointer is the variable position_ (not needed on fpga anymore)
//                        !!! DON'T USE BUFFER_FPGA_POINTER REGISTER ANYMORE !!!
///////////////////////////////////////////////////////////////////////////////////////////////

//initialize static buffer counter
size_t SharedRingBuffer::number_of_buffers_ = 0;

SharedRingBuffer::SharedRingBuffer(const uint32_t data_size, const uint32_t length,
                                   uint32_t registerAddress)
    : data_size_(data_size),
      length_(length),
      position_(length-1),
      registers_(registerAddress, CAM_REGISTER_BYTES),
      buffer_nr_(number_of_buffers_++) {

  if ((fpga_device=open("/dev/slam-sensor", O_RDWR|O_SYNC))<0)
  {
    int myerr = errno;
    printf("ERROR: device open failed (errno %d %s)\n", myerr,
           strerror(myerr));
    exit(-1);
  }

  // get the size of the allocated memory
  read(fpga_device, &allocated_memory_size_, 100);
  printf("Max memory space: %d\n", allocated_memory_size_);

  // allocate shared memory space
  mmap_start_address_ = (char *) mmap(0, allocated_memory_size_,
                                      PROT_READ | PROT_WRITE | PROT_EXEC,
                                      MAP_SHARED, fpga_device,
                                      buffer_nr_ * sysconf(_SC_PAGE_SIZE));
  //  printf("mmap address: 0x%x\n", addr);
  if (mmap_start_address_ == MAP_FAILED) {
    int myerr = errno;
    printf("ERROR: mmap failed (errno %d %s)\n", myerr,
           strerror(myerr));
    exit(-1);
  }

  read(fpga_device, &buffer_physical_address_, buffer_nr_);
  printf("buffer_physical_address_: %p\n", (void*)buffer_physical_address_);

  // init buffer on FPGA side
  registers_.setRegisterValue(BUFFER_ADDRESS, get_start_address());

  //DO NOT SET THIS REGISTER (instead we use the workaround (FPGA-W1) )
  //registers_.setRegisterValue(BUFFER_FPGA_POINTER, get_start_address());

  registers_.setRegisterValue(BUFFER_LENGTH, length);
  registers_.setRegisterValue(BUFFER_PACKAGE_SIZE, data_size);
  registers_.print_register_values();
}

SharedRingBuffer::~SharedRingBuffer() {
}

volatile char* SharedRingBuffer::data() {
  return mmap_start_address_ + getLinuxPointerPosition() * data_size_ + 4;
}

uint32_t SharedRingBuffer::size() const{
  return data_size_ * length_;
}

void SharedRingBuffer::flushBuffer() {
  //uint32_t linux_before = getFpgaPointerPosition();
  //uint32_t fpga_before = getFpgaPointerPosition();

  //set linux pointer to fpga pointer to flush old data
  uint32_t fpga_pointer_pos = getFpgaPointerPosition();
  setLinuxPointerPosition(fpga_pointer_pos);

  //printf(">> Flushing ringbuffer: (B-> fpga: %u, linux: %u / A-> fpga: %u, linux: %u)\n",linux_before, fpga_before, getFpgaPointerPosition(), getLinuxPointerPosition());
}

bool SharedRingBuffer::newDataAvailable() {
  //calculate number of new elements on ring
  uint32_t fpga_pos = getFpgaPointerPosition();
  uint32_t linux_pos = getLinuxPointerPosition();

  uint32_t new_elements = (fpga_pos - linux_pos) % length_;

  if(new_elements>0)
    return true;

  return false;
}

bool SharedRingBuffer::isCurrentDataMostRecent()
{
  // reag fpga pointer
  int fpga_pointer = registers_.getRegisterValue(BUFFER_FPGA_POINTER);

  // compare if fpga is still on the current position or has new data and went forward
  if(buffer_physical_address_ + data_size_*getLinuxPointerPosition() == fpga_pointer)
  {
    // printf("fpga pointer: %p linux pointer: %p\n", fpga_pointer,(void*) buffer_physical_address_ + next_data_position*data_size_);
    // printf("fpga pointer is not on next data: %d == %d\n", (fpga_pointer - buffer_physical_address_)/data_size_, (void*) next_data_position);
    return false;
  }

  if(fpga_pointer == 0)
  {
//    printf("fpga pointer is NULL\n");
    return false;
  }

  // check if fpga pointer is pointing to a valid position
  if((fpga_pointer-buffer_physical_address_)%data_size_ != 0)
  {
    //TODO(schneith): something is wrong with the IMU initialization here, this needs to be fixex at some point
    //                this is just a workaround
    //printf("fpga pointer not valid: points on %f position with: %p\n", (double)(fpga_pointer - buffer_physical_address_)/(double)data_size_, (void*)fpga_pointer);
    return false;
  }
  return true;
}


uint32_t SharedRingBuffer::length() const {
  return length_;
}

uint32_t SharedRingBuffer::data_size() const {
  return data_size_;
}

uint32_t SharedRingBuffer::get_start_address() const{
  return buffer_physical_address_;
}

void SharedRingBuffer::movePointer() {
  // calculate new data position
  uint32_t current_position = getLinuxPointerPosition();
  uint32_t next_position = current_position + 1;

  if(next_position >= length_)
    next_position = 0;

  setLinuxPointerPosition(next_position);
}

void SharedRingBuffer::printfAllData() {
  printf(" ");
  uint32_t used_buffer_size = length_ * data_size_;
static int id = 0;
  printf("---------ID: %u ---F: %u // L: %u----------\n", id++, getFpgaPointerPosition(), getLinuxPointerPosition());
  for(unsigned int i = 0; i<used_buffer_size; i+=data_size_)
  {
    printf("%d: \t", i);

    char* stamp = (char*) (mmap_start_address_ + i + 4);
    uint32_t time= *(uint32_t*)stamp;
    printf(" %u\t", Swap4Bytes(time));

    //datasize
    for(uint32_t j = 0; j<4; j++)
      printf("%.2hhX", (mmap_start_address_[i+j]));

    if(i == getLinuxPointerPosition()*data_size_)
      printf(" L ");
    else
      printf("   ");

    if(i == getFpgaPointerPosition()*data_size_)
      printf(" F \t");
    else
      printf("   \t");

    //Data
    printf("\t");
    for(unsigned int j = 8; j<data_size_; j++)
      printf("%.2hhX", (mmap_start_address_[i+j]));

    printf("\n");

  }
  printf("\n");
}

void SharedRingBuffer::on() {
  //Flush buffer of old data
  flushBuffer();

  //printf("------------ SharedRingBuffer ON (%u) -----------\n", buffer_physical_address_);
  // start moving data to RAM
  registers_.setRegisterValue(CONTROL, 0x01);
}

void SharedRingBuffer::off() {
  // stop FPGA reading camera
  registers_.setRegisterValue(CONTROL, 0x00);

  //printf("------------ SharedRingBuffer OFF (%u) -----------\n", buffer_physical_address_);

  //flush buffer
  flushBuffer();
}

void SharedRingBuffer::print_current_data_size(){
  // change endianess
  char* raw = (char*) (mmap_start_address_ + getLinuxPointerPosition()*data_size_);
  uint32_t current_data_size = Swap4Bytes(*(uint32_t*)raw);

  printf("current data size: %u.\n;", current_data_size);
}

uint32_t SharedRingBuffer::current_data_size(){
  // change endianess
  char* raw = (char*) (mmap_start_address_ + getLinuxPointerPosition()*data_size_);
  uint32_t current_data_size = Swap4Bytes(*(uint32_t*)raw);

  // check if data size is longer than a slot in the buffer
  if(current_data_size > data_size()-4)
  {
    printf("current data size bigger than allowed. Got: %u.\n;", current_data_size);
    current_data_size = data_size()-4;
  }

  return current_data_size;
}

uint32_t SharedRingBuffer::current_timestamp() {

  // change endianess
  char* raw = (char*) (mmap_start_address_ + getLinuxPointerPosition()*data_size_ + 4);
  uint32_t current_timestamp = Swap4Bytes(*(uint32_t*)raw);

  return current_timestamp;
}

void SharedRingBuffer::printPointers() {
  printf("fpga position: %u, linux position:%u\n", getFpgaPointerPosition(), getLinuxPointerPosition());
}

void SharedRingBuffer::setLinuxPointerPosition(uint32_t linux_pointer_pos) {
  boost::unique_lock<boost::shared_mutex> lock(m_position_);

  if(linux_pointer_pos < 0 || linux_pointer_pos > length_)
  {
    printf("[ERROR]: SharedRingBuffer: tried to set invalid linux pointer %u but > length: %u", linux_pointer_pos, length_);
    exit(1);
  }

  position_ = linux_pointer_pos;
}

uint32_t SharedRingBuffer::getLinuxPointerPosition() {
  boost::shared_lock<boost::shared_mutex> lock(m_position_);
  return position_;
}

uint32_t SharedRingBuffer::getFpgaPointerPosition() {

  uint32_t fpga_pointer = registers_.getRegisterValue(BUFFER_FPGA_POINTER);
  uint32_t fpga_pos = (uint32_t)((double)(fpga_pointer - buffer_physical_address_)/(double)data_size_);

  //WORKAROUND FOR FPGA BUG (FPGA-W1)
  //at startup: before any data has been written to ram the fpga pointer register hasn't been written
  //            but is at position 0 (this should only happen once at startup !)
  if(fpga_pointer == 0)
  {
    //printf("fpga pointer register not yet initialized (fpga position: 0)\n");
    return length()-1;
  }

  if(fpga_pos < 0 || fpga_pos > length_)
  {
    printf("[ERROR]: SharedRingBuffer: invalid fpga pointer position: %u (length: %u, pointer: %u)", fpga_pos, length_, fpga_pointer);
    exit(1);
  }

  return fpga_pos;
}
