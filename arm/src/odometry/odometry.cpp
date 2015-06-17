/*
 * odometry.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: samuel
 */

#include "odometry.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////

unsigned int current_frame = 0;

typedef Matrix<float, Dynamic, Dynamic, RowMajor> MatrixType;

MatrixType intensity_data[2];
MatrixType     depth_data[2];

uint32_t intensity_timestamp[2] = {0, 0};
uint32_t     depth_timestamp[2] = {0, 0};

const CameraIntrinsics visensor_intrinsics = CameraIntrinsics(
        /* sensor size     */ Eigen::Vector2f(752, 480),
        /* principal point */ Eigen::Vector2f(370.105, 226.664),
        /* focal length    */ 471.7,
        /* stereo baseline */ 0.110174);

Warp::Parameters minimization_parameters(new ErrorWeightNone());

///////////////////////////////////////////////////////////////////////////////
void initOdometry()
{
    minimization_parameters.pyramid_levels = 3;
    minimization_parameters.max_iterations = 100;
    minimization_parameters.T_init = Transformation(0,0,0,0,0,0);
    minimization_parameters.gradient_norm_threshold = 0; //0.1;
}
///////////////////////////////////////////////////////////////////////////////
void shutdownOdometry()
{
	delete minimization_parameters.weight_function;
}
///////////////////////////////////////////////////////////////////////////////
void handleNewData(const Sensor::Ptr sensor)
{
	if(sensor->getSensorType() == visensor::SensorType::CAMERA_MT9V034) {
		if (sensor->id() == 0) {
			// take data from cam0 (the left? TODO: verify that this is the correct camera!)

			// TODO: do we get column- or row-major data?
			intensity_data[current_frame].resize(FRAME_HEIGHT, FRAME_WIDTH);

			memcpy(intensity_data[current_frame].data(), (const char*) sensor->data_mover()->data(), sizeof(intensity_data));

			intensity_timestamp[current_frame] = sensor->data_mover()->current_timestamp();

			handleFrame();
		}

	} else if(sensor->getSensorType() == visensor::SensorType::DENSE_MATCHER) {

		depth_data[current_frame].resize(FRAME_HEIGHT, FRAME_WIDTH);

		memcpy(depth_data[current_frame].data(), (const char*) sensor->data_mover()->data(), sizeof(depth_data));

		depth_timestamp[current_frame] = sensor->data_mover()->current_timestamp();

		handleFrame();

	} else {
		printf("WARNING: unknown sensor type\n");
	}
}
///////////////////////////////////////////////////////////////////////////////
void handleFrame()
{
	if (intensity_timestamp[current_frame] == depth_timestamp[current_frame]) {
		// got two matching images
		if (intensity_timestamp[1-current_frame] > 0) {
			// actually got two frames

			printf(" >>>>> PROCESSING STEP [ %d -> %d ] <<<<<< \n", intensity_timestamp[1-current_frame], intensity_timestamp[current_frame]);

			CameraImage frame_current, frame_prev;

			frame_current.loadFromMatrices(intensity_data[  current_frame], depth_data[  current_frame]);
			frame_prev   .loadFromMatrices(intensity_data[1-current_frame], depth_data[1-current_frame]);

			CameraStep step(frame_prev, frame_current, visensor_intrinsics);

			findTransformationWithPyramid(step, minimization_parameters);
		}


		// copy current to prev
		current_frame = 1-current_frame;
	}
}
///////////////////////////////////////////////////////////////////////////////
