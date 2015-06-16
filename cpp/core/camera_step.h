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
    CameraStep(const CameraImage& first, const CameraImage& second, const Transformation& ground_truth, const CameraIntrinsics& intrinsics);

    CameraImage frame_first;
    CameraImage frame_second;

    void downsampleBy(unsigned int s);

    unsigned int scale;

    CameraIntrinsics intrinsics;



    // for convenience
    Transformation ground_truth;
    unsigned int index;
    const Scene * scene;
};

#endif /* end of include guard: CAMERA_STEP_H_INCLUDED */
