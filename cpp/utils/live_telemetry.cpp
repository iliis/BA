/*
 * live_telemetry.cpp
 *
 *  Created on: Jun 26, 2015
 *      Author: samuel
 */

#include "live_telemetry.h"

namespace GlobalTiming
{
	// construct stopped timer
#define X(var, tag) timing::Timer var(tag, true);
	GLOBAL_TIMERS
#undef X
};
