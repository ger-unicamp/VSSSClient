#include "strategy/goalkeeper.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

ctrl::vec2 gpk::follow(fira_message::Robot &robot, fira_message::Ball &ball)
{
    ctrl::vec2 pos_robot = ctrl::vec2(robot);
    ctrl::vec2 pos_ball = ctrl::vec2(ball);
    double auxx,auxy;

    if (pos_ball[1] > 0.3 )
    {
        auxx = -3*(pos_robot[0] - -0.65 );
        auxy = 5*(pos_robot[1] - 0.3);
    }
    else if (pos_ball[1] < -0.3)
    {
        auxx = -3*(pos_robot[0] - -0.65 );
        auxy = 5*(pos_robot[1] - -0.3);
    }
    else
    {
        auxx = -3*(pos_robot[0] - -0.65 );
        auxy = 5*(pos_robot[1] - pos_ball[1]);
    }

    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    return apf_vec;

}

ctrl::vec2 gpk::kick(fira_message::Robot &robot)
{
    ctrl::vec2 pos_robot = ctrl::vec2(robot);
    double auxx,auxy;

    if (pos_robot[1] >= 0)
    {
        auxx = -30;
        auxy = 30;
    }
    else
    {
        auxx = 30;
        auxy = -30;
    }

    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    return apf_vec;

    
    
}