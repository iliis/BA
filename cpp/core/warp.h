#ifndef WARP_H_INCLUDED
#define WARP_H_INCLUDED

#include <iostream>
#include <Eigen/Dense>

#include "camera_image.h"
#include "transformation.h"
#include "camera_step.h"
#include "scene.h"
#include "../utils/progress.h"

namespace Warp {

    /*!
     * contains all the interesting math!
     */

    WorldPoint projectInv(const Pixel& pixel,      const CameraIntrinsics& intrinsics);
    Pixel      project   (const WorldPoint& point, const CameraIntrinsics& intrinsics);
    WorldPoint transform (const WorldPoint& point, const Transformation& transformation);

    Eigen::Matrix<float, 2, 3> projectJacobian  (const WorldPoint& point, const CameraIntrinsics& intrinsics);
    Eigen::Matrix<float, 3, 6> transformJacobian(const WorldPoint& point, const Transformation& transformation);
    Eigen::Matrix<float, 1, 2> sampleJacobian   (const Pixel& pixel, const CameraImage& image);

    //float drawError(sf::RenderTarget& target, const CameraStep& step, const Transformation& T);
    float calcError(const CameraStep& step, const Transformation& T, Eigen::VectorXf& error_out, Eigen::Matrix<float, Eigen::Dynamic, 6>& J_out, sf::RenderTarget* plotTarget = NULL);

    struct PlotRange {

        PlotRange(unsigned int dim, float from, float to, unsigned int steps)
         : dim(dim), from(from), to(to), steps(steps) { assert(dim < 6); assert(steps > 0); };

        unsigned int dim;

        // linspace(from, to, steps);
        float from;
        float to;
        unsigned int steps;
    };

    void renderErrorSurface(Eigen::MatrixXf& target_out, Eigen::Matrix<float,Eigen::Dynamic,6>& gradients_out, const CameraStep& step, const Transformation& Tcenter, const PlotRange& range1, const PlotRange& range2);
    /*
    project_inverse();
    transform();
    project();
    sample();

    error, J = warp();
    */

}

#endif /* end of include guard: WARP_H_INCLUDED */
