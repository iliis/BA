//============================================================================
// Name        : slam_sensor_server.cpp
// Author      : Pascal Gohl
// Version     :
// Copyright   : 
// Description :
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/thread.hpp>
#include "tcp_server.hpp"
#include "ConfigServer.hpp"
#include <time.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/timer.hpp>

#include "AutoDiscovery.hpp"
#include "tcp_server.hpp"
#include "ConfigServer.hpp"
#include "ClockSyncServer.hpp"

#include "sensors/MT9V034.hpp"
#include "sensors/Corner.hpp"
#include "sensors/Tau640.hpp"
#include "sensors/ADIS16448.hpp"
#include "sensors/ADIS16488.hpp"
#include "sensors/MPU9150.hpp"
#include "sensors/TimingBlock.hpp"
#include "sensors/LightControl.hpp"
#include "sensors/DenseMatcher.hpp"
#include "sensors/ExternalTrigger.hpp"
#include "sensors/ZynqStatus.hpp"
#include "fpga/FpgaConstants.hpp"
#include "fpga/FPGARegisters.hpp"
#include "calibration/CalibrationStorage.hpp"
#include "serial_bridge/SerialBridge.hpp"

#include "IrqScheduler.hpp"

#include "odometry/odometry.h"
#include "odometry/utils/FakeCamSensor.h"

#define TCP_IMU_PORT 13776
#define TCP_DATA_PORT 13777
#define TCP_CONFIG_PORT 13778
#define TCP_DISCOVERY_PORT 13779
#define TCP_SERIAL_PORT 13780
#define TCP_CLOCK_SYNC_PORT 13781

enum FpgaConfig {
	A, // plain 4 cams
	B, //
	C, // 3 cams and flir
	D, // dense
	UNDEF
};

void printIrqStatisticsLoop(IrqScheduler* sched) {
	while (1) {
		sched->printIrqStatistics();
		sleep(1);
	}
}

int main(void) {

	ZynqStatus status = ZynqStatus(0x78440000);
	ZynqStatus::BitstreamVersion fpga_version = status.getBitstreamVersion();

	FpgaConfig fpga_config = FpgaConfig::UNDEF;
	if (fpga_version.note.compare("version_a       ") == 0) {
		printf("standard image detected..\n");
		fpga_config = FpgaConfig::A;
	} else if (fpga_version.note.compare("version_c       ") == 0) {
		printf("flir image detected.\n");
		fpga_config = FpgaConfig::C;
	} else if (fpga_version.note.compare("version_d       ") == 0) {
		printf("dense image detected.\n");
		fpga_config = FpgaConfig::D;
	} else {
		printf("fpga image not recognised. got '%s'.\n",
				fpga_version.note.c_str());
		fpga_config = FpgaConfig::D;
	}

	/***************************************/
	/* initialize the sensors/fpga cores   */
	/***************************************/
	Sensor::Map sensors;

	if (fpga_config == FpgaConfig::A)
		sensors.insert(
				std::make_pair(visensor::SensorId::EXTERNAL_TRIGGER0,
						boost::make_shared<ExternalTrigger>(
								visensor::SensorId::EXTERNAL_TRIGGER0,
								FPGA_CONSTANTS::EX_TRIGGER_CONFIG,
								FPGA_CONSTANTS::EX_TRIGGER_BUFFER)));

	if (fpga_config != FpgaConfig::D
			&& ADIS16448::checkPresence(FPGA_CONSTANTS::IMU0_ADDRESS_BUS)) {
		sensors.insert(
				std::make_pair(visensor::SensorId::IMU0,
						boost::make_shared<ADIS16448>(visensor::SensorId::IMU0,
								FPGA_CONSTANTS::IMU0_ADDRESS_CONFIG,
								FPGA_CONSTANTS::IMU0_ADDRESS_BUFFER,
								FPGA_CONSTANTS::IMU0_ADDRESS_BUS)));
	} else if (fpga_config != FpgaConfig::D
			&& ADIS16488::checkPresence(FPGA_CONSTANTS::IMU0_ADDRESS_BUS)) {
		sensors.insert(
				std::make_pair(visensor::SensorId::IMU0,
						boost::make_shared<ADIS16488>(visensor::SensorId::IMU0,
								FPGA_CONSTANTS::IMU0_ADDRESS_CONFIG,
								FPGA_CONSTANTS::IMU0_ADDRESS_BUFFER,
								FPGA_CONSTANTS::IMU0_ADDRESS_BUS)));
	} else if (fpga_config
			== FpgaConfig::A /* HACK(gohlp): can be remove when imu switcher works in flir image */) {
		status.setImuConnectorConfiguration(PcbVersion::P2_0);

		if (ADIS16448::checkPresence(FPGA_CONSTANTS::IMU0_ADDRESS_BUS))
			sensors.insert(
					std::make_pair(visensor::SensorId::IMU0,
							boost::make_shared<ADIS16448>(
									visensor::SensorId::IMU0,
									FPGA_CONSTANTS::IMU0_ADDRESS_CONFIG,
									FPGA_CONSTANTS::IMU0_ADDRESS_BUFFER,
									FPGA_CONSTANTS::IMU0_ADDRESS_BUS)));

	} else if (fpga_config
			== FpgaConfig::A /* HACK(gohlp): can be remove when imu switcher works in flir image */) {
		status.setImuConnectorConfiguration(PcbVersion::P2_0);

		if (ADIS16488::checkPresence(FPGA_CONSTANTS::IMU0_ADDRESS_BUS))
			sensors.insert(
					std::make_pair(visensor::SensorId::IMU0,
							boost::make_shared<ADIS16488>(
									visensor::SensorId::IMU0,
									FPGA_CONSTANTS::IMU0_ADDRESS_CONFIG,
									FPGA_CONSTANTS::IMU0_ADDRESS_BUFFER,
									FPGA_CONSTANTS::IMU0_ADDRESS_BUS)));
	}

	// HACK(gohlp): fpga version can be remove when MPU works in flir image
	if (fpga_config == FpgaConfig::A && fpga_version.minor == 0
			&& MPU9150::checkPresence(FPGA_CONSTANTS::MPU0_ADDRESS_BUS))
		sensors.insert(
				std::make_pair(visensor::SensorId::IMU_CAM0,
						boost::make_shared<MPU9150>(
								visensor::SensorId::IMU_CAM0,
								FPGA_CONSTANTS::MPU0_ADDRESS_CONFIG,
								FPGA_CONSTANTS::MPU0_ADDRESS_BUFFER,
								FPGA_CONSTANTS::MPU0_ADDRESS_BUS)));

	// HACK(gohlp): fpga version can be remove when MPU works in flir image
	if (fpga_config == FpgaConfig::A && fpga_version.minor == 0
			&& MPU9150::checkPresence(FPGA_CONSTANTS::MPU1_ADDRESS_BUS))
		sensors.insert(
				std::make_pair(visensor::SensorId::IMU_CAM1,
						boost::make_shared<MPU9150>(
								visensor::SensorId::IMU_CAM1,
								FPGA_CONSTANTS::MPU1_ADDRESS_CONFIG,
								FPGA_CONSTANTS::MPU1_ADDRESS_BUFFER,
								FPGA_CONSTANTS::MPU1_ADDRESS_BUS)));

	//cams
	if (MT9V034::checkPresence(FPGA_CONSTANTS::CAM0_ADDRESS_BUS))
		sensors.insert(
				std::make_pair(visensor::SensorId::CAM0,
						boost::make_shared<MT9V034>(visensor::SensorId::CAM0,
								FPGA_CONSTANTS::CAM0_ADDRESS_CONFIG,
								FPGA_CONSTANTS::CAM0_ADDRESS_BUFFER,
								FPGA_CONSTANTS::CAM0_ADDRESS_BUS)));

	if (MT9V034::checkPresence(FPGA_CONSTANTS::CAM1_ADDRESS_BUS))
		sensors.insert(
				std::make_pair(visensor::SensorId::CAM1,
						boost::make_shared<MT9V034>(visensor::SensorId::CAM1,
								FPGA_CONSTANTS::CAM1_ADDRESS_CONFIG,
								FPGA_CONSTANTS::CAM1_ADDRESS_BUFFER,
								FPGA_CONSTANTS::CAM1_ADDRESS_BUS)));

	if (fpga_config != FpgaConfig::C && fpga_config != FpgaConfig::D
			&& MT9V034::checkPresence(FPGA_CONSTANTS::CAM2_ADDRESS_BUS))
		sensors.insert(
				std::make_pair(visensor::SensorId::CAM2,
						boost::make_shared<MT9V034>(visensor::SensorId::CAM2,
								FPGA_CONSTANTS::CAM2_ADDRESS_CONFIG,
								FPGA_CONSTANTS::CAM2_ADDRESS_BUFFER,
								FPGA_CONSTANTS::CAM2_ADDRESS_BUS)));

	if (fpga_config != FpgaConfig::C && fpga_config != FpgaConfig::D
			&& MT9V034::checkPresence(FPGA_CONSTANTS::CAM3_ADDRESS_BUS))
		sensors.insert(
				std::make_pair(visensor::SensorId::CAM3,
						boost::make_shared<MT9V034>(visensor::SensorId::CAM3,
								FPGA_CONSTANTS::CAM3_ADDRESS_CONFIG,
								FPGA_CONSTANTS::CAM3_ADDRESS_BUFFER,
								FPGA_CONSTANTS::CAM3_ADDRESS_BUS)));

	if (fpga_config == FpgaConfig::C)
		sensors.insert(
				std::make_pair(visensor::SensorId::FLIR3,
						boost::make_shared<Tau640>(visensor::SensorId::FLIR3,
								FPGA_CONSTANTS::FLIR3_ADDRESS_CONFIG,
								FPGA_CONSTANTS::FLIR3_ADDRESS_BUFFER,
								FPGA_CONSTANTS::FLIR3_ADDRESS_BUS)));

	//corner detectors
//  if(MT9V034::checkPresence(FPGA_CONSTANTS::CAM1_ADDRESS_BUS))
//    sensor.push_back(new Corner(visensor::SensorId::CORNER_CAM0, FPGA_CONSTANTS::CORNER0_ADDRESS_CONFIG, FPGA_CONSTANTS::CORNER0_ADDRESS_BUFFER));
//  if(MT9V034::checkPresence(FPGA_CONSTANTS::CAM0_ADDRESS_BUS))
//    sensor.push_back(new Corner(visensor::SensorId::CORNER_CAM1, FPGA_CONSTANTS::CORNER1_ADDRESS_CONFIG, FPGA_CONSTANTS::CORNER1_ADDRESS_BUFFER));

	if (fpga_config == FpgaConfig::D)
		sensors.insert(
				std::make_pair(visensor::SensorId::DENSE_MATCHER0,
						boost::make_shared<DenseMatcher>(
								visensor::SensorId::DENSE_MATCHER0,
								FPGA_CONSTANTS::DENSE_ADDRESS_CONFIG,
								FPGA_CONSTANTS::DENSE_ADDRESS_BUFFER)));

//  sensor.push_back(new ExternalTrigger(visensor::SensorId::EXTERNAL_TRIGGER0, FPGA_CONSTANTS::EX_TRIGGER_CONFIG, FPGA_CONSTANTS::EX_TRIGGER_BUFFER));

	//LED CONTROLLER
	if (fpga_config != FpgaConfig::D) {
		sensors.insert(
				std::make_pair(visensor::SensorId::LED_FLASHER0,
						boost::make_shared<LightControl>(
								visensor::SensorId::LED_FLASHER0,
								FPGA_CONSTANTS::LIGHT_CONTROL_REGCONF)));
	}
	//auxiliary cores

//  sensors.insert(
//      std::make_pair(
//          visensor::SensorId::NOTE_SPECIFIED,
//          boost::make_shared<LightControl>(FPGA_CONSTANTS::LIGHT_CONTROL_REGCONF)));

	boost::shared_ptr<TimingBlock> timing_block = boost::make_shared<TimingBlock>(FPGA_CONSTANTS::TIMING_BLOCK_REGCONF);
	sensors.insert(std::make_pair(visensor::SensorId::SENSOR_CLOCK, timing_block));
	sensors.insert(std::make_pair(visensor::SensorId::SENSOR_CLOCK, boost::make_shared<TimingBlock>(FPGA_CONSTANTS::TIMING_BLOCK_REGCONF)));


	// add virtual camera for debug output
	printf("adding fake camera\n");
	sensors.insert(std::make_pair(visensor::SensorId::CAM2, boost::make_shared<FakeCamSensor>(visensor::SensorId::CAM2)));
	printf("fake camera created\n");


	//turn off all sensors to get the fpga cores in a defined state if the server wasn't shutdown properly

	BOOST_FOREACH(const Sensor::Map::value_type& sensor_pair, sensors)
		sensor_pair.second->off();

	try {
		///////////////////////////////////
		// Start network servers
		///////////////////////////////////

		//initialize the serial bridge server
		SerialBridge serial_br(TCP_SERIAL_PORT);

		//setup network io service
		boost::asio::io_service io_service;

		//timer for connection checks if idle
		boost::asio::deadline_timer t1(io_service, boost::posix_time::milliseconds(100)); //idle check timer


		boost::asio::io_service::work work(io_service); // needed to keep the io_service running between client connections
		boost::thread bt(boost::bind(&boost::asio::io_service::run, &io_service)); // run servers in other threads

		// Autodiscovery responder
		AutoDiscovery discovery_service(io_service, TCP_DISCOVERY_PORT);
		discovery_service.startService();

		// TCP server: streams imu messages
		TcpServer tcp_server(io_service, TCP_DATA_PORT);

		// IMU server: streams imu messages
		TcpServer imu_server(io_service, TCP_IMU_PORT);

		//Load calibration
		CalibrationStorage calib_provider("calibration.xml");

		// handles config messages requests
		ConfigServer config_server(io_service, TCP_CONFIG_PORT, sensors, &calib_provider);
		boost::thread ct(boost::bind(&ConfigServer::run, &config_server));

		// handles clock sync messages
		ClockSyncServer clock_sync_server(io_service, TCP_CLOCK_SYNC_PORT, timing_block);
		boost::thread cs(boost::bind(&ClockSyncServer::run, &clock_sync_server));

//    config_server.run();

		///////////////////////////////////
		// Stream messages
		///////////////////////////////////

		///////////////////////////////////
		// Work loop
		///////////////////////////////////

		initOdometry();

		printf("starting main loop...\n");

		//boost::asio::deadline_timer t2(io_service, boost::posix_time::milliseconds(100)); //idle check timer
		uint32_t framedrop_cnt = 0;
		uint32_t trigger_id = 0;

		bool clientConnected_oldstate = false;
		while (1) {
			///////////////////////////////////
			// Check client connection
			///////////////////////////////////
			bool clientConnected =
					   tcp_server.hasConnection()
					&& imu_server.hasConnection()
					&& config_server.getHostInitialized();

			//react to client disconnect
			if (!clientConnected && clientConnected_oldstate) {
				printf("power off all sensors due to client disconnect\n");
				BOOST_FOREACH(const Sensor::Map::value_type& sensor_pair, sensors)
					sensor_pair.second->off();

				printf("exiting application\n");
				break; // exit application
			}

			///////////////////////////////////
			// Stream messages
			///////////////////////////////////

			//go through all sensors
			BOOST_FOREACH(const Sensor::Map::value_type& sensor_pair, sensors) {

				//only stream the following sensors
				const Sensor::Ptr sensor = sensor_pair.second;
				int sensor_type = sensor->getSensorType();

				// CAM2 is a fake camera to transmit debug data
				if (sensor->id() == 2)
					continue;

				///
				// Sensors that send all available measurements
				///
				if (sensor_type == visensor::SensorType::EXTERNAL_TRIGGER
				 || sensor_type == visensor::SensorType::IMU_ADIS16448
				 || sensor_type == visensor::SensorType::IMU_ADIS16488
				 || sensor_type == visensor::SensorType::MPU_9150) {
//         if(sensor_type == visensor::SensorType::IMU_ADIS16448)
//         {
//
//
//           if( t2.expires_at() <= boost::posix_time::second_clock::local_time() )
//           {
//             system("clear");
//             sensor[i]->data_mover()->printfAllData();
//
//             t2.expires_from_now(boost::posix_time::milliseconds(50));
//           }
//         }

					//check if the sensor has new data
					while (sensor->data_mover()->newDataAvailable()) {

						if (!clientConnected)
							printf("movepointer without client: %u\n",
									sensor->id());

						//move shared buffer pointer (release the measurement)
						sensor->data_mover()->movePointer();

						//send over network if sensor is active and has a client
						if (clientConnected && sensor->isActive()) {
							IpComm::Header header;
							header.timestamp = sensor->data_mover()->current_timestamp();
							header.data_size = sensor->data_mover()->current_data_size();
							header.data_id   = sensor->id();

							imu_server.sendNetworkData(sensor->data_mover()->data(), header);
						}
					}

					///
					// Sensors that send ONE available measurements
					///
				} else if (sensor_type == visensor::SensorType::CAMERA_MT9V034
						|| sensor_type == visensor::SensorType::CORNER_MT9V034
						|| sensor_type == visensor::SensorType::DENSE_MATCHER
						|| sensor_type == visensor::SensorType::CAMERA_TAU640) {
					//check if the sensor has new data
					if (sensor->data_mover()->newDataAvailable()) {
						if (!clientConnected)
							printf("movepointer without client: %u\n", sensor->id());

						//move shared buffer pointer (release the measurement)
						sensor->data_mover()->movePointer();

#if 1
						printf("got new frame! %d bytes at %d ", sensor->data_mover()->current_data_size(), sensor->data_mover()->current_timestamp());

						switch (sensor_type) {
							case visensor::SensorType::CAMERA_MT9V034:
								printf("CAMERA");
								break;

							case visensor::SensorType::DENSE_MATCHER:
								printf("DENSE");
								break;

							default:
								printf("OTHER");
								break;
						}

						printf(" first 4 bytes: %X %X %X %X",
								sensor->data_mover()->data()[0],
								sensor->data_mover()->data()[1],
								sensor->data_mover()->data()[2],
								sensor->data_mover()->data()[3]);

						printf(" sensorID: %d ", sensor->id());

						printf(" pixel[0,0] = %d ", (int) sensor->data_mover()->data()[10000]);

						printf("\n");
#endif



						//drop old frames (if network is too slow)
					    while( sensor->data_mover()->newDataAvailable() )
					    {
						  //printf(">> dropping camera frame on %u / total drops: %u (network too slow?)\n", sensor->id(), framedrop_cnt++);
						  sensor->data_mover()->movePointer();
					    }

						// printf("sensor id: %d send dense size: %u\n", sensor->id(), sensor->data_mover()->current_data_size());

						//send over network if sensor is active and has a client
						if (clientConnected && sensor->isActive()) {
							IpComm::Header header;
							header.timestamp = sensor->data_mover()->current_timestamp();
							// HACK(gohlp) something is wrong with the dense data mover
							//             header.data_size = sensor->data_mover()->current_data_size();
							header.data_size = sensor->data_mover()->data_size();
							header.data_id   = sensor->id();

							tcp_server.sendNetworkData(sensor->data_mover()->data(), header);

							/*
							Transformation t(42,1,0.123,12345,-9999,0.42);
							header.data_size = sizeof(float) * 6;
							header.data_id = 41;
							tcp_server.sendNetworkData((char*) t.value.data(), header);
							 */


							handleNewData(sensor, tcp_server);
						}
					}
				} else if (sensor_type == visensor::SensorType::TIMING_BLOCK) {
					if (timing_block->needToSyncTime()) {
						// send message to signal a time sync slot
						IpComm::Header header_time;
						header_time.timestamp = timing_block->getTime();
						header_time.data_size = 0;
						header_time.data_id = timing_block->id();
						tcp_server.sendNetworkData(0, header_time);
						timing_block->waitForTimeSyncCompletion();
					}
				}

			} //for(unsigned int i = 0; i<sensor.size(); i++)

			clientConnected_oldstate = clientConnected;
		} //while(1)

	} catch (std::exception& e) {
		std::cerr << e.what() << std::endl;
	}

	//turn off all sensors
	printf("power off sensors\n");
	BOOST_FOREACH(const Sensor::Map::value_type& sensor_pair, sensors)
		sensor_pair.second->off();

	shutdownOdometry();

	return 0;
}
