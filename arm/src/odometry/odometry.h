/*
 * odometry.h
 *
 *  Created on: Jun 17, 2015
 *      Author: samuel
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <string.h>
#include <Eigen/Dense>

const unsigned int FRAME_WIDTH  = 752;
const unsigned int FRAME_HEIGHT = 480;

#include "sensors/MT9V034.hpp"
#include "sensors/DenseMatcher.hpp"

#include "core/minimization.h"


///////////////////////////////////////////////////////////////////////////////

void initOdometry();
void shutdownOdometry();

///////////////////////////////////////////////////////////////////////////////

void handleNewData(const Sensor::Ptr sensor);
void handleFrame();

///////////////////////////////////////////////////////////////////////////////

#endif /* ODOMETRY_H_ */
