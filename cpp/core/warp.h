#ifndef WARP_H_INCLUDED
#define WARP_H_INCLUDED

#include <iostream>
#include <Eigen/Dense>

#include "camera_image.h"
#include "transformation.h"
#include "camera_step.h"
#include "scene.h"
#include "../utils/progress.h"
#include "weight_functions.h"

namespace Warp {

    /*!
     * contains all the interesting math!
     */

    struct Parameters {
        ErrorWeightFunction* weight_function;
        float gradient_norm_threshold; // keep only pixels with gradient >= this value
        bool filter_on_unwarped_gradient; // use gradient of image-to-be-warped instead of keyframe at warped points
        unsigned int pyramid_levels;
        Transformation T_init;
        unsigned int max_iterations;

        Parameters(ErrorWeightFunction* weight_function,
                float gradient_norm_threshold = 0,
                bool filter_on_unwarped_gradient = false,
                unsigned int pyramid_levels = 3,
                const Transformation& T_init = Transformation(0,0,0,0,0,0),
                unsigned int max_iterations = 1000)
          : weight_function(weight_function),
            gradient_norm_threshold(gradient_norm_threshold),
            filter_on_unwarped_gradient(filter_on_unwarped_gradient),
            pyramid_levels(pyramid_levels),
            T_init(T_init),
            max_iterations(max_iterations) {}

        void setWeightFunction(ErrorWeightFunction* func) { if (weight_function) { delete weight_function; } weight_function = func; }

        std::string toString();
    };

    WorldPoint projectInv(const Pixel& pixel,      const CameraIntrinsics& intrinsics);
    Pixel      project   (const WorldPoint& point, const CameraIntrinsics& intrinsics);
    WorldPoint transform (const WorldPoint& point, const Transformation& transformation);

    Eigen::Matrix<float, 2, 3> projectJacobian  (const WorldPoint& point, const CameraIntrinsics& intrinsics);
    Eigen::Matrix<float, 3, 6> transformJacobian(const WorldPoint& point, const Transformation& transformation);
    Eigen::Matrix<float, 1, 2> sampleJacobian   (const Pixel& pixel, const CameraImage& image);

    //float drawError(sf::RenderTarget& target, const CameraStep& step, const Transformation& T);
    float calcError(const CameraStep& step, const Transformation& T, Eigen::VectorXf& error_out, Eigen::Matrix<float, Eigen::Dynamic, 6>& J_out, const Parameters& params, sf::RenderTarget* plotTarget = NULL, sf::Font* font = NULL);

    struct PlotRange {

        PlotRange(unsigned int dim, float from, float to, unsigned int steps)
         : dim(dim), from(from), to(to), steps(steps) { assert(dim < 6); assert(steps > 0); };

        unsigned int dim;

        // linspace(from, to, steps);
        float from;
        float to;
        unsigned int steps;

        void readFromStdin();
    };

    void renderErrorSurface(Eigen::MatrixXf& target_out, Eigen::Matrix<float,Eigen::Dynamic,6>& gradients_out, const CameraStep& step, const Transformation& Tcenter, const PlotRange& range1, const PlotRange& range2, const Parameters& params);

}

#endif /* end of include guard: WARP_H_INCLUDED */
