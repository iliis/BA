#ifndef CAMERA_STEP_H_INCLUDED
#define CAMERA_STEP_H_INCLUDED

#include "camera_image.h"
#include "transformation.h"

class Scene;

/*!
 * a single step in a longer trajectory
 * contains two frames (CameraImage)
 */
class CameraStep
{
public:
    CameraStep(const CameraImage& first, const CameraImage& second, const Transformation& ground_truth, const Scene* scene, unsigned int index = 0);

    CameraImage frame_first;
    CameraImage frame_second;

    void downsampleBy(unsigned int s);

    unsigned int scale;

    CameraIntrinsics intrinsics;



    // for convenience
    const Transformation& ground_truth;
    const unsigned int index;
    const Scene * const scene;
};

#endif /* end of include guard: CAMERA_STEP_H_INCLUDED */
