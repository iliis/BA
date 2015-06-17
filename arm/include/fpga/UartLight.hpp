#ifndef _UartLight_h
#define _UartLight_h

#include <stdint.h>
#include <string>

#include "DevMem.h"

class UartLight
{
public:
	UartLight(int register_address);
	~UartLight();

  int writeByte(int8_t data);
  int readByte(int8_t &data);

private:
  void waitForNotFullOutBuffer();
  void waitForNotFullInBuffer();
  void waitForNotEmptyInBuffer();

	DevMem mem;
};

#endif
