#include "minimization_streamlined.h"

using namespace std;
using namespace Eigen;

namespace Streamlined {

///////////////////////////////////////////////////////////////////////////////

// 'static', to prevent any re-allocation of memory
VectorXf errors;
Matrix<float, Dynamic, 6> J;

///////////////////////////////////////////////////////////////////////////////
void initIntrinsicsPyramid(std::vector<CameraIntrinsics>& pyramid, const unsigned int max_level)
{
    for (unsigned int level = 1; level <= max_level; level++) {
        pyramid.push_back(pyramid[level-1]);
        pyramid[level].downsample2();
    }
}
///////////////////////////////////////////////////////////////////////////////
void initImagePyramid(std::vector<MatrixXf>& pyramid, const CameraIntrinsics& intrinsics, const unsigned int max_level)
{
    unsigned int w_p = intrinsics.getCameraWidth();
    unsigned int h_p = intrinsics.getCameraHeight();
    for (unsigned int level = 0; level <= max_level; level++) {

        pyramid.push_back(MatrixXf::Zero(h_p, w_p));

        w_p /= 2;
        h_p /= 2;
    }

}
///////////////////////////////////////////////////////////////////////////////
void buildImagePyramid(std::vector<MatrixXf>& pyramid, const unsigned int max_level)
{
    unsigned int w_p = pyramid[0].cols();
    unsigned int h_p = pyramid[0].rows();
    for (unsigned int level = 1; level <= max_level; level++) {

        // make sure, image has the right size (this is a no-op when already at
        // the correct size)
        w_p /= 2;
        h_p /= 2;
        //pyramid[level].resize(h_p, w_p); // shouldn't be necessary at all

        //cout << "build pyramid level " << level << " with size " << w_p << " x " << h_p << endl;

        // actually do the downsampling
        // TODO: handle invalid values (NaN should be handled automatically)
        for (unsigned int y = 0; y < h_p; y++) {
            for (unsigned int x = 0; x < w_p; x++) {
                pyramid[level](y,x) = (
                        + pyramid[level-1](y*2,   x*2)
                        + pyramid[level-1](y*2+1, x*2)
                        + pyramid[level-1](y*2,   x*2+1)
                        + pyramid[level-1](y*2+1, x*2+1)
                    ) / 4;
            }
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(
        const Eigen::MatrixXf& keyframe_intensities,
        const Eigen::MatrixXf& current_intensities,
        const Eigen::MatrixXf& current_depths,
        Eigen::Matrix<float, 6, 1> &T,
        const CameraIntrinsics& intrinsics,
        const unsigned int level,
        const Warp::Parameters& params)
{
    WarpStreamlined::calcError(
            keyframe_intensities,
            current_intensities,
            current_depths,
            T,
            intrinsics,
            errors, J,
            level,
            params);

    // step = - (J'*W*J) \ J' * W * err';
    VectorXf weights = (*params.weight_function)(errors);

    Matrix<float, 6, 1> delta = - (J.transpose() * weights.asDiagonal() * J).fullPivLu().solve(
            J.transpose() * weights.asDiagonal() * errors );

    //cout << "  delta: " << delta.transpose() << "  norm: " << delta.norm() << endl;

    T += delta;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformationWithPyramid(
        std::vector<MatrixXf>& keyframe_intensities,
        std::vector<MatrixXf>& current_intensities,
        std::vector<MatrixXf>& current_depths,
        const std::vector<CameraIntrinsics>& intrinsics,
        const Warp::Parameters& params)
{
    // calculate image pyramid
    ///////////////////////////////////////////////////////////////////////

    // we assume the keyframe pyramid was already built in previous iteration
    //buildImagePyramid(keyframe_intensities, params.max_pyramid_levels);
    buildImagePyramid(current_intensities, params.max_pyramid_levels);
    buildImagePyramid(current_depths,      params.max_pyramid_levels);

    // actually perform minimization
    ///////////////////////////////////////////////////////////////////////

    Matrix<float, 6, 1> T = params.T_init.value;

    for (int level = params.max_pyramid_levels; level >= (int)params.min_pyramid_levels; level--) {

        float prev_delta = 0;
        float delta = 0;

        unsigned int iterations = 0;

        while (iterations++ < params.max_iterations) {
            prev_delta = delta;
            delta = Streamlined::IterGaussNewton(
                    keyframe_intensities[level],
                    current_intensities[level],
                    current_depths[level],
                    T,
                    intrinsics[level],
                    level,
                    params);

            if (delta < 0.0001) // found good enough solution
                break;

            if (abs(prev_delta - delta) < 0.0001) // convergence rate is decreasing
                break;

            //cout << abs(prev_delta - delta) << " ";
        }

        if (iterations == params.max_iterations)
            cout << "MAX ITER! ";

        //cout << " [ " << level << " ] found solution: " << T.transpose() << endl;
    }

    //cout << " >>>>> found solution: " << T.transpose() << endl;

    return Transformation(T);
}
///////////////////////////////////////////////////////////////////////////////

} // namespace streamlined
