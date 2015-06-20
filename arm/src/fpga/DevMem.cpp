/*
 * DevMem.cpp
 *
 *  Created on: Jul 31, 2013
 *      Author: pascal
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdexcept>

#include "fpga/DevMem.h"

#define GPIO_DATA_OFFSET     0
#define GPIO_DIRECTION_OFFSET     4

#define PAGE_SIZE 4096UL  //ZYNQ page size, found by luck
#define MAP_MASK (PAGE_SIZE - 1)

DevMem::DevMem(off_t offset, size_t size) {
	open_dev(offset, size);
	physical_base = offset;
}

DevMem::~DevMem() {
	// unmap the memory before exiting

	if (munmap(mapped_base, map_size) == -1) {
		printf("Can't unmap memory from user space.\n");
		exit(0);
	}

	close(memfd);
}

void DevMem::open_dev(off_t dev_base, size_t size) {

	map_size = size;

	memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd == -1) {
		printf("Can't open /dev/mem.\n");
		exit(0);
	}
	// printf("/dev/mem opened.\n");

	// Map one page of memory into user space such that the device is in that page, but it may not
	// be at the start of the page.

	mapped_base = (char*)mmap(0, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, dev_base & ~MAP_MASK);

	if (mapped_base == (void *) -1) {
		int myerr = errno;
		printf("Can't map the memory to user space:  (errno %d %s)\n", myerr, strerror(myerr));

		exit(0);
	}
	// printf("Memory mapped at address %p.\n", mapped_base);

	// get the address of the device in user space which will be an offset from the base
	// that was mapped as memory is mapped at the start of a page

	mapped_dev_base = mapped_base + (dev_base & MAP_MASK);
}

void DevMem::write(off_t offset, size_t length,
		unsigned char* data) {

  // check for valid range
  if(offset + length > map_size) {
    printf("DevMem: tried to access data out of range. (offset: %X; length: %X)\n", offset, length );
    throw std::runtime_error("DevMem: tried to access data out of range.");
    exit(-1);
  }

	for(size_t i = 0; i < length; i++)
		*((volatile unsigned char *) (mapped_dev_base + offset + i)) = data[i];
}

int DevMem::read(off_t offset, size_t length, unsigned char* data) {

  // check for valid range
  if(offset + length > map_size) {
    printf("DevMem: tried to access data out of range. (offset: %X; length: %X)\n", offset, length );
    throw std::runtime_error("DevMem: tried to access data out of range.");
    exit(-1);
  }

	for(size_t i = 0; i < length; i++)
		data[i] = *((volatile unsigned char *) (mapped_dev_base + offset + 1));

	return length;
}

void DevMem::write_byte(off_t offset, char data) {

  // check for valid range
  if(offset > map_size) {
    printf("DevMem: tried to access data out of range. (base: %X; offset: %X)\n", physical_base, offset );
    throw std::runtime_error("DevMem: tried to access data out of range.");
    exit(-1);
  }

	*((volatile unsigned char *) (mapped_dev_base + offset)) = data;
}

void DevMem::write_word(off_t offset, off_t data) {

  // check for valid range
  if(offset + 3 > map_size) {
    printf("DevMem: tried to access data out of range. (base: %X; offset: %X)\n", physical_base, offset );
    throw std::runtime_error("DevMem: tried to access data out of range.");
    exit(-1);
  }

	*((volatile unsigned long *) (mapped_dev_base + offset)) = data;
}

char DevMem::read_byte(off_t offset) {

  // check for valid range
  if(offset > map_size) {
    printf("DevMem: tried to access data out of range. (offset: %X)\n", offset);
    throw std::runtime_error("DevMem: tried to access data out of range.");
    exit(-1);
  }

	return *((volatile unsigned char *) (mapped_dev_base + offset));
}

int DevMem::read_word(off_t offset) const{

  // check for valid range
  if(offset + 3 > map_size) {
    printf("DevMem: tried to access data out of range. (offset: %X)\n", offset );
    throw std::runtime_error("DevMem: tried to access data out of range.");
    exit(-1);
  }

	return *((volatile unsigned long *) (mapped_dev_base + offset));
}
