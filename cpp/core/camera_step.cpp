#include "camera_step.h"

///////////////////////////////////////////////////////////////////////////////
CameraStep::CameraStep(const CameraImage& first, const CameraImage& second, const CameraIntrinsics& intrinsics, const Transformation& ground_truth, unsigned int index_first, unsigned int index_second)
  : frame_first(first), frame_second(second),
    scale(0), intrinsics(intrinsics),
    ground_truth(ground_truth),
    index_first(index_first),
    index_second(index_second)
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
