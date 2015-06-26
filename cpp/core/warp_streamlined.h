#ifndef WARP_STREAMLINED_H_INCLUDED
#define WARP_STREAMLINED_H_INCLUDED

#include "image_data.h"
#include "warp.h"
#include "../utils/live_telemetry.h"

namespace WarpStreamlined {
    float calcError(
            const Eigen::MatrixXf& keyframe_intensities,
            const Eigen::MatrixXf& current_intensities,
            const Eigen::MatrixXf& current_depths,
            const Eigen::Matrix<float, 6, 1>& T,
            const CameraIntrinsics& intrinsics,
            Eigen::VectorXf& error_out,
            Eigen::Matrix<float, Eigen::Dynamic, 6>& J_out,
            const float step_scale,
            const Warp::Parameters& params);
}

#endif /* end of include guard: WARP_STREAMLINED_H_INCLUDED */
