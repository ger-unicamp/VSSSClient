#ifndef GPK
#define GPK
#include "strategy/controller.h"


namespace gpk
{
    ctrl::vec2 follow(ctrl::vec2 pos_robot, ctrl::vec2 pos_ball);
    ctrl::vec2 kick(ctrl::vec2 pos_robot, ctrl::vec2 pos_ball);
};


#endif