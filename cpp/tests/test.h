// To add new testcases, simply add a file named test_FOOBAR.cpp and rerun cmake

//Link to Boost
 #define BOOST_TEST_DYN_LINK

//VERY IMPORTANT - include this last
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <iostream>

// cannot use BOOST_CHECK_CLOSE(), as one of the values might be exactly 0

#define CHECK_CLOSE_VECT2(a, b)  do { \
    BOOST_CHECK_SMALL((a).x() - (b).x(), tolerance); \
    BOOST_CHECK_SMALL((a).y() - (b).y(), tolerance); \
} while (0)

#define CHECK_CLOSE_VECT3(a, b)  do { \
    BOOST_CHECK_SMALL((a).x() - (b).x(), tolerance); \
    BOOST_CHECK_SMALL((a).y() - (b).y(), tolerance); \
    BOOST_CHECK_SMALL((a).z() - (b).z(), tolerance); \
} while (0)
