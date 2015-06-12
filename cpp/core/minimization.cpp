#include "minimization.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(const CameraStep& step, Transformation& T)
{

    VectorXf errors;
    Matrix<float, Dynamic, 6> J;

    float total_error = Warp::calcError(step, T, errors, J);

    cout << "[GN] " << T << " --> err: " << total_error;

    // step = - (J'*W*J) \ J' * W * err';

    Matrix<float, 6, 1> delta = - (J.transpose() * J).fullPivLu().solve( J.transpose() * errors );

    cout << "  delta: " << delta.transpose() << "  norm: " << delta.norm() << endl;

    Transformation T_new;
    T_new.value = T.value + delta;
    T_new.updateRotationMatrix();

    T = T_new;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
float IterGradientDescent(const CameraStep& step, Transformation& T, const Matrix<float,6,1>& stepSize)
{
    VectorXf errors;
    Matrix<float, Dynamic, 6> J;

    float total_error = Warp::calcError(step, T, errors, J);

    cout << "[GD] " << T << " --> err: " << total_error;

    // step = - step_size .* (J' * (w.^2 .* err)');

    Matrix<float, 6, 1> delta = -stepSize.array() * (J.transpose() * errors).array();

    cout << "  delta: " << delta.transpose() << endl;

    Transformation T_new;
    T_new.value = T.value + delta;
    T_new.updateRotationMatrix();

    T = T_new;

    return delta.norm();
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformation(const CameraStep& step, Transformation T_init)
{
    Transformation T = T_init;

    float prev_delta = 0;
    float delta = 0;

    int max_iterations = 100;

    while (max_iterations-- > 0) {
        prev_delta = delta;
        delta = IterGaussNewton(step, T);

        if (delta < 0.0001) // found good enough solution
            break;

        if (abs(prev_delta - delta) < 0.0001) // convergence rate is decreasing
            break;

        //cout << abs(prev_delta - delta) << " ";
    }

    cout << " >>>>> found solution: " << T << endl;

    return T;
}
///////////////////////////////////////////////////////////////////////////////
Transformation findTransformationWithPyramid(const CameraStep& step, const unsigned int pyramid_scale)
{
    // downsample images
    CameraStep s = step;
    std::vector<CameraStep> pyramid;
    pyramid.push_back(s);

    for (unsigned int i = 1; i <= pyramid_scale; i++) {
        s.downsampleBy(1);
        pyramid.push_back(s);
    }


    // actually process them
    Transformation T;
    for (std::vector<CameraStep>::const_reverse_iterator it = pyramid.rbegin(); it != pyramid.rend(); ++it) {
        T = findTransformation(*it, T);
    }

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
