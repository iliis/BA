/*
 * Tau640.hpp
 *
 *  Created on: Jul 15, 2013
 *      Author: pascal
 */

#ifndef TAU640_HPP_
#define TAU640_HPP_

#include <string>

#include "fpga/UartLight.hpp"
#include "CamBase.hpp"

#define TAU640_MAX_RATE 30
#define TAU640_MIN_RATE 1

class Tau640: public CamBase {
public:

  enum class CamStatus {
  CAM_OK,
  CAM_BUSY,
  CAM_NOT_READY,
  CAM_RANGE_ERROR,
  CAM_CHECKSUM_ERROR,
  CAM_UNDEFINED_PROCESS_ERROR,
  CAM_UNDEFINED_FUNCTION_ERROR,
  CAM_TIMEOUT_ERROR,
  CAM_BYTE_COUNT_ERROR,
  CAM_FEATURE_NOT_ENABLED,
  UNDEFINED
  };

	Tau640(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress, int bus_address);
	virtual ~Tau640();

	static bool checkPresence(int bus_address);
  virtual int getSensorType(){return visensor::SensorType::CAMERA_TAU640;};
	virtual void on(uint32_t rate);
	void off();

	CamStatus sendSetMessage(int8_t function, int bytes, int8_t data[]);
	CamStatus sendGetMessage(int8_t function, int bytes, int8_t data[]);

	virtual void writeUbiRegister(unsigned char reg_addr, int16_t data);
	virtual void readUbiRegister(unsigned char reg_addr, int16_t& data);

private:
	void turnTestPatternOn();
  void configureFlir();

private:
	UartLight bus_;
};

#endif /* TAU640_HPP_ */
