#ifndef CAMERA_STEP_H_INCLUDED
#define CAMERA_STEP_H_INCLUDED

#include "camera_image.h"
#include "camera_intrinsics.h"
#include "transformation.h"

/*!
 * a single step in a longer trajectory
 * contains two frames (CameraImage)
 */
struct CameraStep
{
    CameraStep(const CameraImage& first, const CameraImage& second, const CameraIntrinsics& intrinsics, const Transformation& ground_truth = Transformation(0,0,0,0,0,0), unsigned int index_first = 0, unsigned int index_second = 0);

    CameraImage frame_first;
    CameraImage frame_second;

    void downsampleBy(unsigned int s);

    unsigned int scale;

    CameraIntrinsics intrinsics;


    // for convenience
    Transformation ground_truth;
    unsigned int index_first;
    unsigned int index_second;
};

#endif /* end of include guard: CAMERA_STEP_H_INCLUDED */
