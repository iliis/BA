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
Odometry::Odometry(TcpServer& tcp_server)
  : visensor_intrinsics(
              /* sensor size     */ Eigen::Vector2f(752, 480),
              /* principal point */ Eigen::Vector2f(370.105, 226.664),
              /* focal length    */ 471.7,
              /* stereo baseline */ 0.110174),
    minimization_parameters(new ErrorWeightNone()),
    tcp_server(tcp_server),
    debug_buffer0(NULL)
{
    printf("initializing odometry...\n");

    minimization_parameters.min_pyramid_levels = 1;
    minimization_parameters.max_pyramid_levels = 4;
    minimization_parameters.max_iterations = 100;
    minimization_parameters.T_init = Transformation(0,0,0,0,0,0);
    minimization_parameters.gradient_norm_threshold = 0.01; //0.1;
    minimization_parameters.use_streamlined = true;

    minimization_parameters.cutout_left   = 20;
    minimization_parameters.cutout_right  = 20;
    minimization_parameters.cutout_top    = 50;
    minimization_parameters.cutout_bottom = 20;

    telemetry.parameters = minimization_parameters;

    debug_buffer0 = (char*) malloc(BUFSIZE);

    if (!debug_buffer0) {
        cerr << "Failed to allocate memory for debug buffer!" << endl;
        exit(0);
    }

    memset(debug_buffer0, 0, BUFSIZE);


    // initialize pyramids (empty)
    ///////////////////////////////////////////////////////////////////////////
    cout << "initialize empty pyramids" << endl;
    intrinsics_pyramid.push_back(visensor_intrinsics);

    cout << "build up pyramids" << endl;
    Streamlined::initIntrinsicsPyramid(intrinsics_pyramid, minimization_parameters.max_pyramid_levels);
    cout << "init images:" << endl;
    Streamlined::initImagePyramid(intensity_data[0], visensor_intrinsics, minimization_parameters.max_pyramid_levels);
    Streamlined::initImagePyramid(intensity_data[1], visensor_intrinsics, minimization_parameters.max_pyramid_levels);
    Streamlined::initImagePyramid(    depth_data[0], visensor_intrinsics, minimization_parameters.max_pyramid_levels);
    Streamlined::initImagePyramid(    depth_data[1], visensor_intrinsics, minimization_parameters.max_pyramid_levels);

    cout << "odometry initialized" << endl;
}
///////////////////////////////////////////////////////////////////////////////
Odometry::~Odometry()
{
    printf("shutting down odometry...\n");

    delete minimization_parameters.weight_function;

    free(debug_buffer0);
}
///////////////////////////////////////////////////////////////////////////////
void Odometry::handleNewData(const Sensor::Ptr sensor)
{
    if(sensor->getSensorType() == visensor::SensorType::CAMERA_MT9V034) {
        if (sensor->id() == 0) {
            // take data from cam0 (the left? TODO: verify that this is the correct camera!)

                        // TODO: do we get column- or row-major data?
            //intensity_data[current_frame].resize(FRAME_HEIGHT, FRAME_WIDTH);



            memcpyCharToMatrix(intensity_data[current_frame][0], (const unsigned char*) sensor->data_mover()->data()+4);
            intensity_data[current_frame][0] /= 255.0f;

            //Streamlined::buildImagePyramid(intensity_data[current_frame], minimization_parameters.max_pyramid_levels);

            intensity_timestamp[current_frame] = sensor->data_mover()->current_timestamp();

            //writeRawDataToFile((const char*) sensor->data_mover()->data(), sensor->data_mover()->data_size(), "intensity_raw.csv");
            //writeMatrixToFile(intensity_data[current_frame], "intensity.csv");
            //printf("got camera image\n");

            handleFrame();
        }

    } else if(sensor->getSensorType() == visensor::SensorType::DENSE_MATCHER) {

        //depth_data[current_frame].resize(FRAME_HEIGHT, FRAME_WIDTH);

        memcpyCharToMatrix(depth_data[current_frame][0], (const unsigned char*) sensor->data_mover()->data()+4);
        depth_data[current_frame][0] /= 6.0f;

        //Streamlined::buildImagePyramid(depth_data[current_frame], minimization_parameters.max_pyramid_levels);

        depth_timestamp[current_frame] = sensor->data_mover()->current_timestamp();

        //writeRawDataToFile((const char*) sensor->data_mover()->data(), sensor->data_mover()->data_size(), "depth_raw.csv");
        //writeMatrixToFile(depth_data[current_frame], "depth.csv");
        //printf("got depth image\n");

        handleFrame();

    } else {
        printf("WARNING: unknown sensor type\n");
    }
}
///////////////////////////////////////////////////////////////////////////////
void Odometry::handleFrame()
{
    if (intensity_timestamp[current_frame] == depth_timestamp[current_frame]) {
        // got two matching images
        if (intensity_timestamp[1-current_frame] > 0) {
            // actually got two frames

            //printf(" >>>>> PROCESSING STEP [ %d -> %d ] <<<<<< \n", intensity_timestamp[1-current_frame], intensity_timestamp[current_frame]);

            /*
            CameraImage frame_current, frame_prev;

            frame_current.loadFromMatrices(intensity_data[  current_frame], depth_data[  current_frame]);
            frame_prev   .loadFromMatrices(intensity_data[1-current_frame], depth_data[1-current_frame]);

            CameraStep step(frame_prev, frame_current, visensor_intrinsics);

            telemetry.transformation = findTransformationWithPyramid(step, minimization_parameters);
            */

            telemetry.transformation = Streamlined::findTransformationWithPyramid(
                    intensity_data[1-current_frame], // keyframe
                    intensity_data[  current_frame], // current frame
                        depth_data[  current_frame],
                    intrinsics_pyramid,
                    minimization_parameters);

            cout << telemetry.transformation << endl;

            // copy timestamp
            memcpy(debug_buffer0, (const void*) &intensity_timestamp[current_frame], 4);

            // copy telemetry data
            memcpy(debug_buffer0+4, (const void*) &telemetry, sizeof(Telemetry));

            // send movement telemetry to client
            IpComm::Header header;
            header.timestamp = intensity_timestamp[current_frame];
            header.data_size = BUFSIZE;
            header.data_id   = 2;
            tcp_server.sendNetworkData(debug_buffer0, header);
        }


        // copy current to prev
        current_frame = 1-current_frame;
    }
}
///////////////////////////////////////////////////////////////////////////////
