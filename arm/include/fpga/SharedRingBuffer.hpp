/*
 * shared_memory.hpp
 *
 *  Created on: Jun 17, 2013
 *      Author: pascal
 */

#ifndef SHARED_RING_BUFFER_HPP_
#define SHARED_RING_BUFFER_HPP_

#include <stdint.h>
#include <boost/thread/shared_mutex.hpp>

#include "FPGARegisters.hpp"

#define Swap4Bytes(val) ( (((val) >> 24) & 0x000000FF) | (((val) >>  8) & 0x0000FF00) |    (((val) <<  8) & 0x00FF0000) | (((val) << 24) & 0xFF000000) )

class SharedRingBuffer {
 public:
  SharedRingBuffer(const uint32_t data_size, const uint32_t length, uint32_t registerAddress);
  virtual ~SharedRingBuffer();

  uint32_t get_start_address() const;
  volatile char * data();
  uint32_t size() const;
  uint32_t data_size() const;
  uint32_t current_data_size();
  void print_current_data_size();
  uint32_t current_timestamp();
  uint32_t length() const;
  bool newDataAvailable();
  void flushBuffer();
  void movePointer();

  // checks if current buffer is the newest one or if we should further
  // iterate pointer
  bool isCurrentDataMostRecent();

  //buffer pointer
  uint32_t getFpgaPointerPosition();
  uint32_t getLinuxPointerPosition();
  void setLinuxPointerPosition(uint32_t linux_pointer_pos);

  //turn datamover core on/off
  void on();
  void off();

  //debug functions
  void printfAllData();
  void printPointers();

 private:
  uint32_t data_size_;
  uint32_t length_;
  uint32_t position_;
  boost::shared_mutex m_position_;

  uint32_t allocated_memory_size_;
  uint32_t fpga_device;

  uint32_t buffer_physical_address_;
  volatile char * mmap_start_address_;

  FPGARegisters registers_;

  const size_t buffer_nr_;
  static size_t number_of_buffers_;  //unique buffer counter
};

#endif /* SHARED_RING_BUFFER_HPP_ */
