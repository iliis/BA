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
    CameraStep(const CameraImage& first, const CameraImage& second, const Transformation& ground_truth, unsigned int index = 0, const Scene* scene = NULL);

    const CameraImage& frame_first;
    const CameraImage& frame_second;

    const Transformation& ground_truth;

    // for convenience
    const unsigned int index;
    const Scene * const scene;
};

#endif /* end of include guard: CAMERA_STEP_H_INCLUDED */
