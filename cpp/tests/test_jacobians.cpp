//Define our Module name (prints at testing)
// define this FIRST!
#define BOOST_TEST_MODULE "Jacobians"

#include "test.h"

///////////////////////////////////////////////////////////////////////////////

#include "../core/warp.h"

using namespace std;
using namespace Eigen;

const float tolerance = 1e-4;

///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(jacobian_project)
{
    // values are calculated by Matlab (not checked by hand!)

    CameraIntrinsics intrinsics(7,5,1);

    Eigen::Matrix<float, 2, 3> J = Warp::projectJacobian(WorldPoint(1,1,1), intrinsics);

    Eigen::Matrix<float, 2, 3> J_solution;
    J_solution << 1, 0, -1,
                  0, 1, -1;

    CHECK_CLOSE_MATRIX(J, J_solution);


    J = Warp::projectJacobian(WorldPoint(4,3,2), intrinsics);

    J_solution << 0.5,   0, -1,
                    0, 0.5, -0.75;

    CHECK_CLOSE_MATRIX(J, J_solution);
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(jacobian_transform)
{
    Eigen::Matrix<float, 3, 6> J = Warp::transformJacobian(WorldPoint(1,1,1), Transformation(0,0,0,0,0,0));
    Eigen::Matrix<float, 3, 6> J_solution;
    J_solution << 1, 0, 0,  0,  1, -1,
                  0, 1, 0, -1,  0,  1,
                  0, 0, 1,  1, -1,  0;

    CHECK_CLOSE_MATRIX(J, J_solution);

    J = Warp::transformJacobian(WorldPoint(4,3,2), Transformation(0,0,0,0,0,0));
    J_solution << 1, 0, 0,  0,  2, -3,
                  0, 1, 0, -2,  0,  4,
                  0, 0, 1,  3, -4,  0;

    CHECK_CLOSE_MATRIX(J, J_solution);

    J = Warp::transformJacobian(WorldPoint(4,3,2), Transformation(1,2,3,0.1,0.2,0.3));

    J_solution <<
        1.0000,         0,         0,         0,    1.3771,   -3.9674,
             0,    1.0000,         0,   -1.7743,    0.3268,    2.8398,
             0,         0,    1.0000,    3.8904,   -3.2573,    1.0932;

    CHECK_CLOSE_MATRIX(J, J_solution);
}
///////////////////////////////////////////////////////////////////////////////
