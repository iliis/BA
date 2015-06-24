/*
 * ZynqStatus.hpp
 *
 *  Created on: Dec 10, 2013
 *      Author: schneith
 */

#ifndef ZynqStatus_HPP_
#define ZynqStatus_HPP_

#include "Sensor.hpp"

#include <string>

typedef enum{
  P2_0,
  P2_1
}PcbVersion;

class ZynqStatus : public Sensor {
 public:
	ZynqStatus(int register_address);
  virtual ~ZynqStatus();

  virtual int getSensorType(){ return visensor::SensorType::ZYNQ_STATUS;};

  /* Zynq die temperature */
  float getCurrentTempCelsius(void);
  float getMaxTempCelsius(void);
  float getMinTempCelsius(void);

  void setImuConnectorConfiguration(PcbVersion);

  /* Loaded bitstream version */
  struct BitstreamVersion {
	  unsigned int major;
	  unsigned int minor;
	  unsigned int patch;
	  std::string note;
  };
  BitstreamVersion getBitstreamVersion(void);

 private:
  float readTemperatureRegister(off_t reg_offset);

};

#endif /* ZynqStatus_HPP_ */
