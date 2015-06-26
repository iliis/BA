#include "minimization.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
// TODO: clean this up, too lazy to implement this cleanly :P
int iteration_count = 0;

//#define PRINT_DEBUG_MESSAGES

// 'static', to prevent any re-allocation of memory
VectorXf errors;
Matrix<float, Dynamic, 6> J;

// TODO: clean this up, too lazy to implement this cleanly :P
float valid_percentage = 0;
float bad_percentage = 0;
bool was_bad_last_step = false;

///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(const CameraStep& step, Transformation& T, const Warp::Parameters& params)
{
    ++iteration_count;

    if (params.use_streamlined) {
        valid_percentage = WarpStreamlined::calcError(step.frame_first.intensities.data, step.frame_second.intensities.data, step.frame_second.depths.data, T.value, step.intrinsics, errors, J, step.scale, params);
    } else {
        valid_percentage = Warp::calcError(step, T, errors, J, params);
    }

#ifdef PRINT_DEBUG_MESSAGES
    cout << "[GN] " << T << " --> err: " << errors.norm();
#endif

    // step = - (J'*W*J) \ J' * W * err';
    VectorXf weights = (*params.weight_function)(errors);

    Matrix<float, 6, 1> delta = - (J.transpose() * weights.asDiagonal() * J).fullPivLu().solve(
            J.transpose() * weights.asDiagonal() * errors );

#ifdef PRINT_DEBUG_MESSAGES
    cout << "  delta: " << delta.transpose() << "  norm: " << delta.norm() << endl;
#endif

    Transformation T_new;
    T_new.value = T.value + delta;
    T_new.updateRotationMatrix();

    T = T_new;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
float IterGradientDescent(const CameraStep& step, Transformation& T, const Matrix<float,6,1>& stepSize, const Warp::Parameters& params)
{
    ++iteration_count;

    valid_percentage = Warp::calcError(step, T, errors, J, params);

#ifdef PRINT_DEBUG_MESSAGES
    cout << "[GD] " << T << " --> err: " << errors.norm();
#endif

    // step = - step_size .* (J' * (w.^2 .* err)');
    VectorXf weights = (*params.weight_function)(errors);

    // there is probably a better way than using two diagonal matrices ;) (asArray() or something)
    Matrix<float, 6, 1> delta = -stepSize.array() * (J.transpose() * weights.asDiagonal() * weights.asDiagonal() * errors).array();

#ifdef PRINT_DEBUG_MESSAGES
    cout << "  delta: " << delta.transpose() << endl;
#endif

    Transformation T_new;
    T_new.value = T.value + delta;
    T_new.updateRotationMatrix();

    T = T_new;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformation(const CameraStep& step, Warp::Parameters& params)
{
    Transformation T = params.T_init;

    float prev_delta = 0;
    float delta = 0;

    unsigned int iterations = 0;

    while (iterations++ < params.max_iterations) {
        prev_delta = delta;
        delta = IterGaussNewton(step, T, params);

        if (valid_percentage < params.valid_pixel_threshold) {
            //cout << "WARN: temporarily setting gradient norm threshold to 0" << endl;
            // let's hope this first iteration didn't went astray too extremely...
            params.gradient_norm_threshold = 0;
            bad_percentage = valid_percentage;
        }

        if (delta < 0.0001) // found good enough solution
            break;

        if (abs(prev_delta - delta) < 0.0001) // convergence rate is decreasing
            break;

        //cout << abs(prev_delta - delta) << " ";
    }

    if (iterations == params.max_iterations)
        cout << "MAX ITER! ";

#ifdef PRINT_DEBUG_MESSAGES
    cout << " >>>>> found solution: " << T << endl;
#endif

    return T;
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformationWithPyramid(const CameraStep& step, const Warp::Parameters& params)
{
    // downsample images
    CameraStep s = step;
    std::vector<CameraStep> pyramid;
    pyramid.push_back(s);

#if 0
    boost::timer::cpu_timer timer;
#endif



    for (unsigned int i = 1; i <= params.max_pyramid_levels; i++) {
        s.downsampleBy(1);
        pyramid.push_back(s);
    }

    iteration_count = 0;

    // actually process them
    Warp::Parameters tmp_params = params;

    // if previous call to find transformation already had problems with too
    // few pixels, use all of them from the start and use less downscaling
    if (was_bad_last_step) {
        was_bad_last_step = false;
        tmp_params.gradient_norm_threshold = 0;
        if (tmp_params.max_pyramid_levels > tmp_params.min_pyramid_levels)
            tmp_params.max_pyramid_levels--;
    }

    for (int level = params.max_pyramid_levels; level >= (int)params.min_pyramid_levels; level--) {
        tmp_params.T_init = findTransformation(pyramid[level], tmp_params);
    }

#if 0
    timer.stop();

    // TODO: put this in a debug struct or something
    cout << " >>>>> found solution in " << iteration_count << " iterations and " << timer.format();
    cout << ((double) (timer.elapsed().user/1000))/1000 << "ms ( " << 1000*1000/((double)timer.elapsed().user/1000) << " FPS)" << endl;
    cout << " >>>>> this is " << timer.elapsed().user / ((double) iteration_count * 1000 * 1000) << "ms per Iteration (on average, with " << params.pyramid_levels << " pyramid levels)" << endl;
#else
//#ifdef PRINT_DEBUG_MESSAGES
    cout << " >>>>> found in " << iteration_count << " iterations: " << tmp_params.T_init
        << " valid : " << (valid_percentage*100) << "%";

    if (tmp_params.gradient_norm_threshold != params.gradient_norm_threshold)
        cout << " G_THRESH! (" << bad_percentage << ")";

    cout << endl;
//#endif
#endif

    if (tmp_params.gradient_norm_threshold != params.gradient_norm_threshold) {
        was_bad_last_step = true;
    }


    return tmp_params.T_init;
}
///////////////////////////////////////////////////////////////////////////////
