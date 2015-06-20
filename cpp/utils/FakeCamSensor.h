/*
 * FakeCamSensor.h
 *
 *  Created on: Jun 20, 2015
 *      Author: samuel
 */

#ifndef FAKECAMSENSOR_H_
#define FAKECAMSENSOR_H_

#include "sensors/Sensor.hpp"
#include "visensor/visensor_constants.hpp"

class FakeCamSensor: public Sensor {
public:
	FakeCamSensor(visensor::SensorId::SensorId sensor_id)
	 : Sensor(sensor_id, FPGA_CONSTANTS::CAM2_ADDRESS_CONFIG, CAM_REGISTER_BYTES) {};

	virtual ~FakeCamSensor() {};

	virtual int getSensorType(){return visensor::SensorType::CAMERA_MT9V034;};



	virtual int getCamBaseType(){return 0;};
	virtual void on(uint32_t rate){};
	virtual void off(){};

	virtual void writeUbiRegister(unsigned char reg_addr, int16_t data){};
	virtual void readUbiRegister(unsigned char reg_addr, int16_t& data){};

	virtual void writeFpgaRegister(unsigned char reg_addr, int32_t data) {};
	virtual void readFpgaRegister(unsigned char reg_addr, int32_t& data) {};
};

#endif /* FAKECAMSENSOR_H_ */
