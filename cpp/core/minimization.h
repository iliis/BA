#ifndef MINIMIZATION_H_INCLUDED
#define MINIMIZATION_H_INCLUDED

#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "transformation.h"
#include "camera_step.h"
#include "warp.h"
#include "weight_functions.h"

// perform a single Gauss-Newton step
float IterGaussNewton    (const CameraStep& step, Transformation& T,                                           const Warp::Parameters& params);
float IterGradientDescent(const CameraStep& step, Transformation& T, const Eigen::Matrix<float,6,1>& stepSize, const Warp::Parameters& params);

Transformation findTransformation           (const CameraStep& step, const Warp::Parameters& params);
Transformation findTransformationWithPyramid(const CameraStep& step, const Warp::Parameters& params);

std::vector<Transformation> findTrajectory(const Scene& scene, const Warp::Parameters& params);

#endif /* end of include guard: MINIMIZATION_H_INCLUDED */
