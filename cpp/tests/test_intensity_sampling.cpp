//Define our Module name (prints at testing)
 #define BOOST_TEST_MODULE "Intensity Sampling"

#include "test.h"

///////////////////////////////////////////////////////////////////////////////

#include "../core/image_data.h"

using namespace std;

// tolerance for float comparisons
// must be quite high, as we sample very close to the edge of the image
const float tolerance = 1e-4;

///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE( sample_values )
{
    Eigen::MatrixXf img_data(4,6);
    img_data <<
         1,  2,  3,  4,  5,  6,
        10, 20, 30, 40, 50, 60,
        20, 40, 60, 80,100, 10,
         8,  1,  1,  1,  1,  9;

    ImageData img;
    img.loadFromMatrix(img_data);

    // sample the four corners
    // [1,1]' is at top left
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(0, 0)), 1, tolerance);
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(5, 0)), 6, tolerance);
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(0, 3)), 8, tolerance);
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(5, 3)), 9, tolerance);

    // sample inside image
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(0.5, 0.5)),  8.25, tolerance); // mean([1,2,10,20])   =  8.25
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(1.5, 1.5)), 37.5, tolerance); // mean([20,30,40,60]) = 37.5
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(3.8,   0)),  4.8, tolerance); // 4.8
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(3.8,   1)), 48, tolerance); // 48
    BOOST_CHECK_CLOSE(img.sampleValue(Eigen::Vector2f(1,   1.8)), 36, tolerance); // 36
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE( sample_gradients )
{
    Eigen::MatrixXf img_data(4,6);
    img_data <<
         1,  2,  3,  4,  5,  6,
        10, 20, 30, 40, 50, 60,
        20, 40, 60, 80,100, 10,
         8,  1,  1,  1,  1,  9;

    ImageData img;
    img.loadFromMatrix(img_data);

    // dI/dx = (I(x+d) - I(x)) / d
    // in contrast to the Matlab code we don't pad the gradients with 0!
    // also, values falling exactly on an integer coordinate are interpolated with the right pixel
    // -> this leads to small errors (but hopefully insignificant)
    const float eps = 0.0000004;
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(0, 0)),         Eigen::Vector2f(1,9));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(5-eps, 0)),     Eigen::Vector2f(1,54));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(0, 3-eps)),     Eigen::Vector2f(-7,-12));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(5-eps, 3-eps)), Eigen::Vector2f(8,-1));

    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(0.5, 0.5)), Eigen::Vector2f( 5.5,  13.5));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(1.5, 1.5)), Eigen::Vector2f(15  ,  25));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(3.8,   0)), Eigen::Vector2f( 1  ,  43.2));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(3.8,   1)), Eigen::Vector2f(10  ,  48));
    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(1,   1.8)), Eigen::Vector2f(18  ,  20));
}
///////////////////////////////////////////////////////////////////////////////
BOOST_AUTO_TEST_CASE( sample_gradients_simple )
{
    Eigen::MatrixXf img_data(2,2);
    img_data << 1, 2,
               10,20;
    ImageData img; img.loadFromMatrix(img_data);

    CHECK_CLOSE_VECT2(img.sampleDiff(Eigen::Vector2f(0.5, 0.5)), Eigen::Vector2f(5.5, 13.5));
}
///////////////////////////////////////////////////////////////////////////////
