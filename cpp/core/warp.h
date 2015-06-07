#ifndef WARP_H_INCLUDED
#define WARP_H_INCLUDED

#include <iostream>
#include <Eigen/Dense>

#include "camera_image.h"
#include "transformation.h"
#include "camera_step.h"
#include "scene.h"

namespace Warp {

    /*!
     * contains all the interesting math!
     */

    WorldPoint projectInv(const Pixel& pixel,   const CameraIntrinsics& intrinsics);
    Pixel      project(const WorldPoint& point, const CameraIntrinsics& intrinsics);
    WorldPoint transform(const WorldPoint& point, const Transformation& transformation);

    void drawError(sf::RenderTarget& target, const CameraStep& step, const Transformation& T);
    /*
    project_inverse();
    transform();
    project();
    sample();

    error, J = warp();
    */

}

#endif /* end of include guard: WARP_H_INCLUDED */
