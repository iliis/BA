#ifndef LIVE_TELEMETRY_H_INCLUDED
#define LIVE_TELEMETRY_H_INCLUDED

#include "../core/warp.h"

struct Telemetry
{
    uint32_t magic_constant1;

    Warp::Parameters parameters;
    Transformation   transformation;

    uint32_t magic_constant2;
    // ...
    //

    Telemetry()
     : magic_constant1(0xDEADBEE1),
       parameters(NULL),
       magic_constant2(0xDEADBEE2)
    {};

    bool checkOK() { return magic_constant1 == 0xDEADBEE1 && magic_constant2 == 0xDEADBEE2; }
};

#endif /* end of include guard: LIVE_TELEMETRY_H_INCLUDED */
