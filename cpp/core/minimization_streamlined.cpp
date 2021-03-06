#include "minimization_streamlined.h"

using namespace std;
using namespace Eigen;

namespace Streamlined {

///////////////////////////////////////////////////////////////////////////////

// 'static', to prevent any re-allocation of memory
VectorXf errors;
Matrix<float, Dynamic, 6> J;

// TODO: clean this up, too lazy to implement this cleanly :P
float valid_percentage = 0;
float bad_percentage = 0;
bool was_bad_last_step = false;

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
    valid_percentage =
    WarpStreamlined::calcError(
            keyframe_intensities,
            current_intensities,
            current_depths,
            T,
            intrinsics,
            errors, J,
            level,
            params);

    GlobalTiming::gaussnewton.Start();

    // step = - (J'*W*J) \ J' * W * err';
#if 1
    VectorXf weights = (*params.weight_function)(errors);

    Matrix<float, 6, 1> delta = - (J.transpose() * weights.asDiagonal() * J).fullPivLu().solve(
            J.transpose() * weights.asDiagonal() * errors );
#else
    Matrix<float, 6, 1> delta = - (J.transpose() * J).ldlt().solve(J.transpose() * errors);
#endif

    //cout << "  delta: " << delta.transpose() << "  norm: " << delta.norm() << endl;

    T += delta;

    GlobalTiming::gaussnewton.Stop();

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
    Warp::Parameters tmp_params = params;

    // calculate image pyramid
    ///////////////////////////////////////////////////////////////////////

    GlobalTiming::pyramid.Start();

    // we assume the keyframe pyramid was already built in previous iteration
    //buildImagePyramid(keyframe_intensities, params.max_pyramid_levels);
    buildImagePyramid(current_intensities, params.max_pyramid_levels);
    buildImagePyramid(current_depths,      params.max_pyramid_levels);

    GlobalTiming::pyramid.Stop();

    ///////////////////////////////////////////////////////////////////////

    // if previous call to find transformation already had problems with too
    // few pixels, use all of them from the start and use less downscaling
    if (was_bad_last_step) {
        was_bad_last_step = false;
        tmp_params.gradient_norm_threshold = 0;
        if (tmp_params.max_pyramid_levels > tmp_params.min_pyramid_levels)
            tmp_params.max_pyramid_levels--;
    }

    // actually perform minimization
    ///////////////////////////////////////////////////////////////////////

    Matrix<float, 6, 1> T = params.T_init.value;

    unsigned int total_iterations = 0;

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
                    tmp_params);

            if (valid_percentage < params.valid_pixel_threshold && tmp_params.gradient_norm_threshold > 0) {
                //cout << "WARN: temporarily setting gradient norm threshold to 0: valid = " << valid_percentage*100 << "%" << endl;
                // let's hope this first iteration didn't went astray too extremely...
            	bad_percentage = valid_percentage;
                tmp_params.gradient_norm_threshold = 0;
                was_bad_last_step = true;
            }

            if (delta < 0.0001) // found good enough solution
                break;

            if (abs(prev_delta - delta) < 0.0001) // convergence rate is decreasing
                break;

            //cout << abs(prev_delta - delta) << " ";
        }

        if (iterations == params.max_iterations)
            cout << "MAX ITER! ";

        total_iterations += iterations;

        //cout << " [ " << level << " ] found solution: " << T.transpose() << endl;
    }

    cout << " >>>>> found solution: " << T.transpose() << " in " << total_iterations << " iters.";

    if (tmp_params.gradient_norm_threshold != params.gradient_norm_threshold) {
        cout << " G_THRESH (" << bad_percentage*100 << "%)";
    }

    cout << endl;


    return Transformation(T);
}
///////////////////////////////////////////////////////////////////////////////

} // namespace streamlined
