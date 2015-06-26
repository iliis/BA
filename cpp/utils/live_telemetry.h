#ifndef LIVE_TELEMETRY_H_INCLUDED
#define LIVE_TELEMETRY_H_INCLUDED

#include "../core/warp.h"
#include "timing/timer.hpp"



#define GLOBAL_TIMERS \
	X(frame_total,			"0_Full_Frame") \
	X(odometry_total,		"1_DenseOdometry") \
	X(pyramid,				"1__1 buildPyramids") \
	X(warping,				"1__2 warpTotal") \
	X(warping_rotation,		"1__2__1 calcRotation") \
	X(warping_jacobian_init,"1__2__2 initJacobian") \
	X(warping_warping,		"1__2__3 warping") \
	X(warping_sampling,		"1__2__4 sampling") \
	X(warping_jacobians,	"1__2__5 warpJacobians") \
	X(gaussnewton,			"1__3 GaussNewton") \
	\
	X(tcp, 					"2_TCP")


namespace GlobalTiming
{
#define X(var, tag) extern timing::Timer var;
GLOBAL_TIMERS
#undef X
};


struct Telemetry
{
	// guard
    uint32_t magic_constant1;

    Warp::Parameters parameters;
    Transformation   transformation;

    // guard
    uint32_t magic_constant2;

    Telemetry()
     : magic_constant1(0xDEADBEE1),
       parameters(NULL),
       magic_constant2(0xDEADBEE2)
    {};

    bool checkOK() { return magic_constant1 == 0xDEADBEE1 && magic_constant2 == 0xDEADBEE2; }

}  __attribute__((__aligned__(8))); // ensure proper size on 32 and 64bit systems

#endif /* end of include guard: LIVE_TELEMETRY_H_INCLUDED */
