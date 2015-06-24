//Define our Module name (prints at testing)
// define this FIRST!
#define BOOST_TEST_MODULE "transformation"

#include "test.h"

///////////////////////////////////////////////////////////////////////////////

#include "../core/warp.h"

using namespace std;
using namespace Eigen;

const float tolerance = 1e-6;

///////////////////////////////////////////////////////////////////////////////

BOOST_AUTO_TEST_CASE(no_transformation)
{
    // generate some random points
    for (int i = 10000; i > 0; --i) {
        WorldPoint p;
        p.pos = Vector3f::Random();

        // transform using null transformation
        WorldPoint p_new = Warp::transform(p, Transformation(0,0,0,0,0,0));

        // they should remain untouched
        CHECK_CLOSE_VECT3(p.pos, p_new.pos);
    }
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(translation_only)
{
    Transformation T(1,2,3,0,0,0);

    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 0, 0, 0), T).pos, Vector3f(1, 2, 3));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 1, 0, 0), T).pos, Vector3f(2, 2, 3));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 0, 1, 0), T).pos, Vector3f(1, 3, 3));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 0, 0, 1), T).pos, Vector3f(1, 2, 4));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 1, 1, 1), T).pos, Vector3f(2, 3, 4));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(-1, 0, 1), T).pos, Vector3f(0, 2, 4));
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(rotation_X90)
{
    Transformation T(0,0,0,M_PI/2,0,0);

    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 0, 0 ), T).pos, Vector3f(0, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(1, 0, 0 ), T).pos, Vector3f(1, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 0, -1), T).pos, Vector3f(0, 1, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 1, 0 ), T).pos, Vector3f(0, 0, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(1, 1, -1), T).pos, Vector3f(1, 1, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(-1, 1, 0), T).pos, Vector3f(-1, 0, 1));
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(rotation_Y90)
{
    Transformation T(0,0,0,0,M_PI/2,0);

    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 0, 0, 0 ), T).pos, Vector3f(0, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 0, 0, 1 ), T).pos, Vector3f(1, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint( 0, 1, 0 ), T).pos, Vector3f(0, 1, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(-1, 0, 0 ), T).pos, Vector3f(0, 0, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(-1, 1, 1 ), T).pos, Vector3f(1, 1, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(-1, 0, -1), T).pos, Vector3f(-1, 0, 1));
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(rotation_Z90)
{
    Transformation T(0,0,0,0,0,M_PI/2);

    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 0, 0 ), T).pos, Vector3f(0, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, -1, 0), T).pos, Vector3f(1, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(1, 0, 0 ), T).pos, Vector3f(0, 1, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 0, 1 ), T).pos, Vector3f(0, 0, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(1, -1, 1), T).pos, Vector3f(1, 1, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 1, 1 ), T).pos, Vector3f(-1, 0, 1));
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE(rotation_X90Y90)
{
    Transformation T(0,0,0,M_PI/2,M_PI/2,0);

    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 0, 0 ), T).pos, Vector3f(0, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 0, 1 ), T).pos, Vector3f(1, 0, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(1, 0, 0 ), T).pos, Vector3f(0, 1, 0));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 1, 0 ), T).pos, Vector3f(0, 0, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(1, 1, 1 ), T).pos, Vector3f(1, 1, 1));
    CHECK_CLOSE_VECT3(Warp::transform(WorldPoint(0, 1, -1), T).pos, Vector3f(-1, 0, 1));
}
///////////////////////////////////////////////////////////////////////////////
