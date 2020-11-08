#include "strategy/goalkeeper.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

ctrl::vec2 gpk::follow(fira_message::Robot &robot, fira_message::Ball &ball)
{
    ctrl::vec2 pos_robot = ctrl::vec2(robot);
    ctrl::vec2 pos_ball = ctrl::vec2(ball);
    double auxx,auxy;

    if (pos_ball.y > 0.3 )
    {
        auxx = -2*(pos_robot.x - -0.65 );
        auxy = -6*(pos_robot.y - 0.3);
    }
    else if (pos_ball.y < -0.3)
    {
        auxx = -2*(pos_robot.x - -0.65 );
        auxy = -6*(pos_robot.y - -0.3);
    }
    else
    {
        auxx = -2*(pos_robot.x - -0.65 );
        auxy = -6*(pos_robot.y - pos_ball.y);
    }


    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    return apf_vec;

}

ctrl::vec2 gpk::kick(fira_message::Robot &robot,fira_message::Ball &ball)
{
    ctrl::vec2 pos_robot = ctrl::vec2(robot);
    ctrl::vec2 pos_ball = ctrl::vec2(ball);
    double auxx,auxy;

    if (pos_robot.y < pos_ball.y)
    {
        auxx = 30;
        auxy = -30;
    }
    else
    {
        auxx = -30;
        auxy = 30;
    }

    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    return apf_vec;

    
    
}