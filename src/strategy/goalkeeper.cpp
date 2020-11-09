#include "strategy/goalkeeper.h"
#include <iostream>


ctrl::vec2 gpk::follow(fira_message::Robot &robot, fira_message::Ball &ball)
{
    ctrl::vec2 pos_robot = ctrl::vec2(robot);
    ctrl::vec2 pos_ball = ctrl::vec2(ball);
    double auxx,auxy;

    if (pos_robot.x < -0.73)
    {
        auxx = -(pos_robot.x - -0.65 );
        auxy = -(pos_robot.y - pos_ball.y);
    }
    else if (pos_ball.y > 0.3 )
    {
        auxx = -(pos_robot.x - -0.65 );
        auxy = -(pos_robot.y - 0.3);
    }
    else if (pos_ball.y < -0.3)
    {
        auxx = -(pos_robot.x - -0.65 );
        auxy = -(pos_robot.y - -0.3);
    }
    else
    {
        auxx = -(pos_robot.x - -0.65 );
        auxy = -(pos_robot.y - pos_ball.y);
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
        auxx = 50;
        auxy = -50;
    }
    else
    {
        auxx = -50;
        auxy = 50;
    }

    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    return apf_vec;

    
    
}