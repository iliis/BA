#include "camera_step.h"
#include "scene.h"

///////////////////////////////////////////////////////////////////////////////
CameraStep::CameraStep(const CameraImage& first, const CameraImage& second, const Transformation& ground_truth, unsigned int index, Scene* scene)
  : frame_first(first), frame_second(second),
    ground_truth(ground_truth),
    index(index), scene(scene)
{
}
///////////////////////////////////////////////////////////////////////////////
