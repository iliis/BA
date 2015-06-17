#ifndef _UnifiedBusInterface_h
#define _UnifiedBusInterface_h

#include <stdint.h>
#include <string>

#include "DevMem.h"

class UnifiedBusInterface
{
public:
	UnifiedBusInterface(int register_address, unsigned char device_address);
//	UnifiedBusInterface(unsigned char dev_addr, std::string i2cfilename);

	~UnifiedBusInterface();

	int write16bReg(unsigned char reg_addr, int16_t data);
	int read16bReg(unsigned char reg_addr, int16_t &data);
  int write8bReg(unsigned char reg_addr, int8_t data);
  int read8bReg(unsigned char reg_addr, int8_t &data);
  int read16bRegSPI(unsigned char reg_addr, int16_t &data);
  void waitForNotFullOutBuffer();
  void waitForNotFullInBuffer();
  void waitForNotEmptyInBuffer();

private:

	DevMem mem;
	unsigned char deviceAddress; // i2c device address
};

#endif
