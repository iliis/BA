/*
 * DevMem.h
 *
 *  Created on: Jul 31, 2013
 *      Author: pascal
 */

#ifndef DEVMEM_H_
#define DEVMEM_H_

#include <sys/types.h>

class DevMem {
public:
	DevMem(off_t offset, size_t size);
	virtual ~DevMem();

	void write(off_t offset, size_t length, unsigned char * data);
	void write_byte(off_t offset, char data);
	void write_word(off_t offset, off_t data);
	int read(off_t offset, size_t length, unsigned char * data);
	char read_byte(off_t offset);
	int read_word(off_t offset) const;

private:
	void open_dev(off_t base_address, size_t size);

private:
	int memfd;
	char *mapped_dev_base;
	char *mapped_base;
	off_t physical_base; // only used for debugging
	size_t map_size;
};

#endif /* DEVMEM_H_ */
