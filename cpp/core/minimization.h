#ifndef MINIMIZATION_H_INCLUDED
#define MINIMIZATION_H_INCLUDED

#include <Eigen/Dense>
#include <math.h>
#include <vector>

#include "transformation.h"
#include "camera_step.h"
#include "warp.h"

// perform a single Gauss-Newton step
float IterGaussNewton    (const CameraStep& step, Transformation& T);
float IterGradientDescent(const CameraStep& step, Transformation& T, const Eigen::Matrix<float,6,1>& stepSize);

Transformation findTransformation(const CameraStep& step);

std::vector<Transformation> findTrajectory(const Scene& scene);

#endif /* end of include guard: MINIMIZATION_H_INCLUDED */
