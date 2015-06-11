#include "minimization.h"

using namespace std;
using namespace Eigen;

///////////////////////////////////////////////////////////////////////////////
float IterGaussNewton(const CameraStep& step, Transformation& T)
{

    VectorXf errors;
    Matrix<float, Dynamic, 6> J;

    float total_error = Warp::calcError(step, T, errors, J);

    cout << T << " --> err: " << total_error;

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

    cout << T << " --> err: " << total_error;

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
