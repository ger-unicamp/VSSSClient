#ifndef GPK
#define GPK
#include "strategy/controller.h"


namespace gpk
{
    ctrl::vec2 follow(fira_message::Robot &robot, fira_message::Ball &ball);
    ctrl::vec2 kick(fira_message::Robot &robot);
};


#endif