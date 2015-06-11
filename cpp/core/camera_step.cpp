#include "camera_step.h"
#include "scene.h"

///////////////////////////////////////////////////////////////////////////////
CameraStep::CameraStep(const CameraImage& first, const CameraImage& second, const Transformation& ground_truth, const Scene* scene, unsigned int index)
  : frame_first(first), frame_second(second),
    scale(0), intrinsics(scene->getIntrinsics()),
    ground_truth(ground_truth),
    index(index), scene(scene)
{
}
///////////////////////////////////////////////////////////////////////////////
void CameraStep::downsampleBy(unsigned int s)
{
    this->scale += s;
    while (s-- > 0) {
        this->frame_first.downsample2();
        this->frame_second.downsample2();
        this->intrinsics.downsample2();
    }
}
///////////////////////////////////////////////////////////////////////////////
