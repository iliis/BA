/*
 * ZynqStatus.cpp
 *
 *  Created on: Dec 10, 2013
 *      Author: schneith
 *
 */

#include "sensors/ZynqStatus.hpp"

//see http://www.xilinx.com/support/documentation/ip_documentation/axi_xadc/v1_00_a/pg019_axi_xadc.pdf
//for register and module descriptions

// Example usage + module base address
//=====================================
//	#include "ZynqStatus.hpp"
//	ZynqStatus status(0, 0x78440000);
//
//	while(1)
//	{
//		std::cout << "Temperature (curr / min / max) [deg C]: " <<
//					status.getCurrentTempCelsius() << " / " <<
//					status.getMinTempCelsius()     << " / " <<
//					status.getMaxTempCelsius()     << "\n";
//
//		ZynqStatus::BitstreamVersion version = status.getBitstreamVersion();
//
//		std::cout << "Bitstream Ver. (major.minor.patch / note): " <<
//					version.major << "." <<
//					version.minor << "." <<
//					version.patch << " / " <<
//					version.note  << "\n\n";
//		sleep(1);
//	}

ZynqStatus::ZynqStatus(int register_address)
: Sensor(visensor::SensorId::SENSOR_STATUS, register_address, 0x2000){
}

ZynqStatus::~ZynqStatus() {
}

void ZynqStatus::setImuConnectorConfiguration(PcbVersion version) {
  switch(version){
    case PcbVersion::P2_0:
      config_registers_.setRegisterValue(R_ZSTATUS_IMU_CONNECTOR_CONFIG, 0x01);
      break;
    case PcbVersion::P2_1:
    default:
      config_registers_.setRegisterValue(R_ZSTATUS_IMU_CONNECTOR_CONFIG, 0x00);
      break;
  }
}

float ZynqStatus::readTemperatureRegister(off_t reg_offset) {

	//read the temperature register
	unsigned int word = config_registers_.getRegisterValue(reg_offset);

	//temperature is only 12bit MSB
	unsigned int tempRaw = ( word >> 4 ) & 0xFFF;

	//convert raw temperature to °C
	// temp (C°) = (ADC Code * 503.975) / 4096 - 273.15
	float tempCel = (static_cast<float>(tempRaw) * 503.975) / 4096.0 - 273.15;

	return tempCel;
}

float ZynqStatus::getCurrentTempCelsius(void) {
	return readTemperatureRegister(R_ZSTATUS_TEMP_CURRENT);
}

float ZynqStatus::getMaxTempCelsius(void) {
	return readTemperatureRegister(R_ZSTATUS_TEMP_MAX);
}

float ZynqStatus::getMinTempCelsius(void) {
	return readTemperatureRegister(R_ZSTATUS_TEMP_MIN);
}

ZynqStatus::BitstreamVersion ZynqStatus::getBitstreamVersion(void)
{
	ZynqStatus::BitstreamVersion version;

	/* bitstream version register */
	version.major = config_registers_.getRegisterValue(R_ZSTATUS_BIT_VER_MAJOR);
	version.minor = config_registers_.getRegisterValue(R_ZSTATUS_BIT_VER_MINOR);
	version.patch = config_registers_.getRegisterValue(R_ZSTATUS_BIT_VER_PATCH);

	/* bitstream version note (4x 32 bit) */
	version.note.clear();

	for(size_t i = 0; i<4; i++)
	{
		/* read register */
		unsigned int word = config_registers_.getRegisterValue(R_ZSTATUS_BIT_VER_NOTE + i*4);

		/* add 4 bytes per register */
		char* symbols = reinterpret_cast<char*>( &word );

		for(size_t j = 0; j<4; j++)
		{
			/* only add printable symbols */
			if(symbols[3-j]>=0x20 || symbols[3-j] <= 0x7e)
				version.note += symbols[3-j];
			else
				version.note += '.';
		}
	}

	return version;
}

