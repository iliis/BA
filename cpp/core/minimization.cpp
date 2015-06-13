#include "minimization.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
// TODO: clean this up, too lazy to implement this cleanly :P
int iteration_count = 0;

//#define PRINT_DEBUG_MESSAGES

///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(const CameraStep& step, Transformation& T, const ErrorWeightFunction& weight_func)
{
    ++iteration_count;

    VectorXf errors;
    Matrix<float, Dynamic, 6> J;

    float total_error = Warp::calcError(step, T, errors, J);

#ifdef PRINT_DEBUG_MESSAGES
    cout << "[GN] " << T << " --> err: " << total_error;
#endif

    // step = - (J'*W*J) \ J' * W * err';
    VectorXf weights = weight_func(errors);

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
float IterGradientDescent(const CameraStep& step, Transformation& T, const Matrix<float,6,1>& stepSize, const ErrorWeightFunction& weight_func)
{
    ++iteration_count;

    VectorXf errors;
    Matrix<float, Dynamic, 6> J;

    float total_error = Warp::calcError(step, T, errors, J);

#ifdef PRINT_DEBUG_MESSAGES
    cout << "[GD] " << T << " --> err: " << total_error;
#endif

    // step = - step_size .* (J' * (w.^2 .* err)');
    VectorXf weights = weight_func(errors);

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
Transformation findTransformation(const CameraStep& step, Transformation T_init, const ErrorWeightFunction& weight_func)
{
    Transformation T = T_init;

    float prev_delta = 0;
    float delta = 0;

    int max_iterations = 1000;

    while (max_iterations-- > 0) {
        prev_delta = delta;
        delta = IterGaussNewton(step, T, weight_func);

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
Transformation findTransformationWithPyramid(const CameraStep& step, const unsigned int pyramid_levels, const ErrorWeightFunction& weight_func)
{
    // downsample images
    CameraStep s = step;
    std::vector<CameraStep> pyramid;
    pyramid.push_back(s);

    sf::Clock clock;
    clock.restart();

    for (unsigned int i = 1; i < pyramid_levels; i++) {
        s.downsampleBy(1);
        pyramid.push_back(s);
    }

    iteration_count = 0;

    // actually process them
    Transformation T;
    for (std::vector<CameraStep>::const_reverse_iterator it = pyramid.rbegin(); it != pyramid.rend(); ++it) {
        T = findTransformation(*it, T, weight_func);
    }

    sf::Time t = clock.getElapsedTime();
    cout << " >>>>> found solution in " << iteration_count << " iterations and " << ((double) t.asMicroseconds())/1000 << "ms ( " << 1000*1000/((double)t.asMicroseconds()) << " FPS)" << endl;
    cout << " >>>>> this is " << t.asMicroseconds() / ((double) iteration_count * 1000) << "ms per Iteration (on average, with " << pyramid_levels << " pyramid levels)" << endl;


    return T;
}
///////////////////////////////////////////////////////////////////////////////
std::vector<Transformation> findTrajectory(const Scene& scene)
{
    std::vector<Transformation> traj(scene.getStepCount());

    for (size_t i = 0; i < scene.getStepCount(); i++) {
        traj[i] = findTransformation(scene.getStep(i));
    }

    return traj;
}
///////////////////////////////////////////////////////////////////////////////
