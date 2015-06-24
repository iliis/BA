#ifndef MINIMIZATION_STREAMLINED_H_INCLUDED
#define MINIMIZATION_STREAMLINED_H_INCLUDED


#include <Eigen/Dense>
#include <math.h>
#include <vector>
//#include <boost/timer/timer.hpp>

#include "camera_step.h"
#include "camera_intrinsics.h"
#include "warp.h"
#include "warp_streamlined.h"
#include "weight_functions.h"

namespace Streamlined
{
    void initIntrinsicsPyramid(std::vector<CameraIntrinsics>& pyramid, const unsigned int max_level);

    void initImagePyramid(std::vector<Eigen::MatrixXf>& pyramid, const CameraIntrinsics& intrinsics, const unsigned int max_level);
    void buildImagePyramid(std::vector<Eigen::MatrixXf>& pyramid, const unsigned int max_level);

    float IterGaussNewton(
            const Eigen::MatrixXf& keyframe_intensities,
            const Eigen::MatrixXf& current_intensities,
            const Eigen::MatrixXf& current_depths,
            Eigen::Matrix<float, 6, 1> &T,
            const CameraIntrinsics& intrinsics,
            const unsigned int level,
            const Warp::Parameters& params);

    // this function assumes, keyframe_intensities[] already contains a (previously) downscaled image pyramid!
    Transformation findTransformationWithPyramid(
            std::vector<Eigen::MatrixXf>& keyframe_intensities,
            std::vector<Eigen::MatrixXf>& current_intensities,
            std::vector<Eigen::MatrixXf>& current_depths,
            const std::vector<CameraIntrinsics>& intrinsics,
            const Warp::Parameters& params);
}

#endif /* end of include guard: MINIMIZATION_STREAMLINED_H_INCLUDED */
