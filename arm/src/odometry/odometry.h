/*
 * odometry.h
 *
 *  Created on: Jun 17, 2015
 *      Author: samuel
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <string.h>

#include "sensors/MT9V034.hpp"
#include "sensors/DenseMatcher.hpp"

#include "tcp_server.hpp"
#include "ConfigServer.hpp"

#include "core/minimization.h"
#include "utils/matrix.h"
#include "utils/live_telemetry.h"

///////////////////////////////////////////////////////////////////////////////

const unsigned int FRAME_WIDTH  = 752;
const unsigned int FRAME_HEIGHT = 480;

///////////////////////////////////////////////////////////////////////////////

class Odometry
{
public:

	Odometry(TcpServer& tcp_server);
	~Odometry();

	void handleNewData(const Sensor::Ptr sensor);
	void handleFrame();

private:
	unsigned int current_frame = 0;

	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixType;

	MatrixType intensity_data[2];
	MatrixType     depth_data[2];

	uint32_t intensity_timestamp[2] = {0, 0};
	uint32_t     depth_timestamp[2] = {0, 0};

	Warp::Parameters minimization_parameters;

	// TODO: read intrinsics from calib_provider
	CameraIntrinsics visensor_intrinsics;


	// for debugging
	////////////////////////////////////////////

	Telemetry telemetry;

	const size_t BUFSIZE = 360964;
	char* debug_buffer0;

	TcpServer& tcp_server;
};

///////////////////////////////////////////////////////////////////////////////

#endif /* ODOMETRY_H_ */
