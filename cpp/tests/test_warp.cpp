
//Define our Module name (prints at testing)
// define this FIRST!
#define BOOST_TEST_MODULE "Warp"

#include "test.h"

///////////////////////////////////////////////////////////////////////////////

#include "../core/warp.h"

using namespace std;

const float tolerance = 1e-5;

///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE( project_and_inv )
{
    CameraIntrinsics intrinsics(100, 50, 1.5);


    // generate some random pixels and depth values
    for (int i = 0; i < 1000; i++) {

        //points = rand(2,N) * 100 - 50;
        //depths = rand(1,N) * 100 - 50;
        //WorldPoint projectInv(const Pixel& pixel,      const CameraIntrinsics& intrinsics);
        //Pixel      project   (const WorldPoint& point, const CameraIntrinsics& intrinsics);

        // create a random pixel
        Pixel pixel;
        pixel.pos.setRandom();
        pixel.intensity = Eigen::Vector2f::Random().x();
        pixel.depth     = Eigen::Vector2f::Random().x();

        // project into world
        WorldPoint point_world = Warp::projectInv(pixel, intrinsics);

        // and back onto image
        Pixel pixel_rep = Warp::project(point_world, intrinsics);

        // we should get the original points back (minus some numeric error)
        CHECK_CLOSE_VECT2(pixel.pos, pixel_rep.pos);
    }
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE( project_manual_tests )
{
    // tolerance for float comparisons (in percentage)
    WorldPoint point_world;

    CameraIntrinsics ci(7, 5, 1);

    CHECK_CLOSE_VECT2(ci.getCameraSize(), Eigen::Vector2f(7, 5));
    BOOST_CHECK_CLOSE(ci.getFocalLength(), 1, tolerance);

    // make sure the principal point is where we expect him
    CHECK_CLOSE_VECT2(ci.getPrincipalPoint(), Eigen::Vector2f(3, 2));

    // center pixel (should be equal to principal point)
    point_world = Warp::projectInv(Pixel(3, 2, 10), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(0, 0, 10));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(3, 2));

    // top left pixel
    point_world = Warp::projectInv(Pixel(0, 0, 10), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(-30, -20, 10));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(0, 0));

    // top right pixel
    point_world = Warp::projectInv(Pixel(6, 0, 2), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(6, -4, 2));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(6, 0));

    // bottom left pixel
    point_world = Warp::projectInv(Pixel(0, 4, 10), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(-30, 20, 10));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(0, 4));

    // bottom right pixel
    point_world = Warp::projectInv(Pixel(6, 4, 10), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(30, 20, 10));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(6, 4));

    // a bit right of center
    point_world = Warp::projectInv(Pixel(4, 2, 10), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(10, 0, 10));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(4, 2));

    // a bit above center
    point_world = Warp::projectInv(Pixel(3, 1, 10), ci);
    CHECK_CLOSE_VECT3(point_world.pos, Eigen::Vector3f(0, -10, 10));
    CHECK_CLOSE_VECT2(Warp::project(point_world, ci).pos, Eigen::Vector2f(3, 1));
}
///////////////////////////////////////////////////////////////////////////////
