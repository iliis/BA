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

///////////////////////////////////////////////////////////////////////////////

const unsigned int FRAME_WIDTH  = 752;
const unsigned int FRAME_HEIGHT = 480;

///////////////////////////////////////////////////////////////////////////////

void initOdometry();
void shutdownOdometry();

///////////////////////////////////////////////////////////////////////////////

void handleNewData(const Sensor::Ptr sensor, TcpServer& tcp_server);
void handleFrame();

///////////////////////////////////////////////////////////////////////////////

#endif /* ODOMETRY_H_ */
