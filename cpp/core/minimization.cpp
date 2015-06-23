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

///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(const CameraStep& step, Transformation& T, const Warp::Parameters& params)
{
    ++iteration_count;

    if (params.use_streamlined) {
#ifdef PRINT_DEBUG_MESSAGES
        float total_error = WarpStreamlined::calcError(step.frame_first.intensities.data, step.frame_second.intensities.data, step.frame_second.depths.data, T.value, step.intrinsics, errors, J, step.scale, params);
        cout << "[GN sl] " << T << " --> err: " << total_error;
#else
        WarpStreamlined::calcError(step.frame_first.intensities.data, step.frame_second.intensities.data, step.frame_second.depths.data, T.value, step.intrinsics, errors, J, step.scale, params);
#endif
    } else {
#ifdef PRINT_DEBUG_MESSAGES
        float total_error = Warp::calcError(step, T, errors, J, params);
        cout << "[GN] " << T << " --> err: " << total_error;
#else
        Warp::calcError(step, T, errors, J, params);
#endif
    }

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

#ifdef PRINT_DEBUG_MESSAGES
    float total_error = Warp::calcError(step, T, errors, J, params);
    cout << "[GD] " << T << " --> err: " << total_error;
#else
    Warp::calcError(step, T, errors, J, params);
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
Transformation findTransformation(const CameraStep& step, const Warp::Parameters& params)
{
    Transformation T = params.T_init;

    float prev_delta = 0;
    float delta = 0;

    unsigned int iterations = 0;

    while (iterations++ < params.max_iterations) {
        prev_delta = delta;
        delta = IterGaussNewton(step, T, params);

        if (delta < 0.0001) // found good enough solution
            break;

        if (abs(prev_delta - delta) < 0.0001) // convergence rate is decreasing
            break;

        //cout << abs(prev_delta - delta) << " ";
    }

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


    cout << step.intrinsics << endl;

#if 0
    boost::timer::cpu_timer timer;
#endif

    for (unsigned int i = 1; i < params.pyramid_levels; i++) {
        s.downsampleBy(1);
        pyramid.push_back(s);
    }

    iteration_count = 0;

    // actually process them
    Warp::Parameters p = params;
    for (std::vector<CameraStep>::const_reverse_iterator it = pyramid.rbegin(); it != pyramid.rend(); ++it) {
        p.T_init = findTransformation(*it, p);
    }

#if 0
    timer.stop();

    // TODO: put this in a debug struct or something
    cout << " >>>>> found solution in " << iteration_count << " iterations and " << timer.format();
    cout << ((double) (timer.elapsed().user/1000))/1000 << "ms ( " << 1000*1000/((double)timer.elapsed().user/1000) << " FPS)" << endl;
    cout << " >>>>> this is " << timer.elapsed().user / ((double) iteration_count * 1000 * 1000) << "ms per Iteration (on average, with " << params.pyramid_levels << " pyramid levels)" << endl;
#else
//#ifdef PRINT_DEBUG_MESSAGES
    cout << " >>>>> found solution in " << iteration_count << " iterations: " << p.T_init << endl;
//#endif
#endif


    return p.T_init;
}
///////////////////////////////////////////////////////////////////////////////
