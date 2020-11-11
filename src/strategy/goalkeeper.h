#ifndef GPK
#define GPK
#include "util/mathutil.h"
#include "strategy/controller.h"
#include "strategy/APF.h"

namespace gpk
{
    ctrl::vec2 follow(ctrl::vec2 pos_robot, ctrl::vec2 pos_ball, double k);
    ctrl::vec2 kick(bool cw);
};


#endif