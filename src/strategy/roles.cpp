#include "strategy/roles.h"
#include "goalkeeper.h"
#include <iostream>

ctrl::vec2 rol::goalkeeper(fira_message::Robot &robot, fira_message::Ball &ball)
{      
    ctrl::vec2 apf_vec;

    if (ctrl::vec2(robot).distance(ball) < 0.08)
    {
        apf_vec = gpk::kick(robot, ball);
    }
    else
    {
        apf_vec = gpk::follow(robot, ball);
        apf_vec = ctrl::move_robot(robot,apf_vec,0.300637,40*0.661463+10);
    }

    return apf_vec;
}