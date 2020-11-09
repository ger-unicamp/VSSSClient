#include "strategy/goalkeeper.h"
#include <iostream>


ctrl::vec2 gpk::follow(ctrl::vec2 pos_robot, ctrl::vec2 pos_ball)
{
    double auxx,auxy;

    if (pos_robot.x < -0.73)
    {
        auxx = -5*(pos_robot.x - -0.68 );
        auxy = -(pos_robot.y - pos_ball.y);
    }
    else if (pos_ball.y > 0.3 && pos_ball.x > -0.55)
    {
        auxx = -(pos_robot.x - -0.63 );
        auxy = -(pos_robot.y - 0.3);
    }
    else if (pos_ball.y < -0.3 && pos_ball.x > -0.55)
    {
        auxx = -(pos_robot.x - -0.63 );
        auxy = -(pos_robot.y - -0.3);
    }
    else
    {
        
        auxx = -(pos_robot.x - -0.68 );
        auxy = -(pos_robot.y - pos_ball.y);
        
    }

    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    return apf_vec;

}

ctrl::vec2 gpk::kick(ctrl::vec2 pos_robot, ctrl::vec2 pos_ball)
{
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