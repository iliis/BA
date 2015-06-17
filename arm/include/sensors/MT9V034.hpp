/*
 * MT9V034.hpp
 *
 *  Created on: Jul 15, 2013
 *      Author: pascal
 */

#ifndef MT9V034_HPP_
#define MT9V034_HPP_

#include <string>

#include "fpga/UnifiedBusInterface.hpp"
#include "sensors/CamBase.hpp"

#define MT9V034_MAX_RATE 30
#define MT9V034_MIN_RATE 1

class MT9V034: public CamBase {
public:
	MT9V034(visensor::SensorId::SensorId sensor_id, int ctrl_address, int bufferAddress, int bus_address);
	virtual ~MT9V034();

	static bool checkPresence(int bus_address);
    virtual int getSensorType(){return visensor::SensorType::CAMERA_MT9V034;};
	virtual void on(uint32_t rate);
	void off();

	virtual void writeUbiRegister(unsigned char reg_addr, int16_t data);
	virtual void readUbiRegister(unsigned char reg_addr, int16_t& data);

private:
	UnifiedBusInterface bus_;
};

#endif /* MT9V034_HPP_ */
