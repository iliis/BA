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
float IterGaussNewton    (const CameraStep& step, Transformation& T, const ErrorWeightFunction& weight_func = ErrorWeightNone());
float IterGradientDescent(const CameraStep& step, Transformation& T, const Eigen::Matrix<float,6,1>& stepSize, const ErrorWeightFunction& weight_func = ErrorWeightNone());

Transformation findTransformation(const CameraStep& step, Transformation T_init = Transformation(), const ErrorWeightFunction& weight_func = ErrorWeightNone());
Transformation findTransformationWithPyramid(const CameraStep& step, const unsigned int pyramid_levels, const ErrorWeightFunction& weight_func = ErrorWeightNone());

std::vector<Transformation> findTrajectory(const Scene& scene);

#endif /* end of include guard: MINIMIZATION_H_INCLUDED */
