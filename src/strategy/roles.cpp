#include "strategy/roles.h"
#include "goalkeeper.h"
#include "APF.h"
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
        apf_vec = ctrl::move_robot(robot,apf_vec,0.300637,40*0.661463+20);
    }

    return 6*apf_vec;
}

ctrl::vec2 rol::attacker(fira_message::Robot &robot, fira_message::Ball &ball)
{
    ctrl::vec2 apf_vec;
    //G0:0.0922878 G1:0.00563102 G2:0.438424 G3: 40 * 0.617454 + 10)
    apf_vec = apf::ball_field(robot, ball, 0.0755485, 0.0691405);
    apf_vec = ctrl::move_robot(robot, apf_vec, 0.443467, 40.0 * 0.664899 + 10.0);

    return apf_vec;
}

ctrl::vec2 rol::defender(fira_message::Robot &robot, fira_message::Ball &ball, int i)
{
    ctrl::vec2 pos_robot = ctrl::vec2(robot);
    ctrl::vec2 pos_ball = ctrl::vec2(ball);
    double auxx,auxy;

    if (i == 0)
    {
        if (pos_ball.x > 0.5)
        {
            auxx = -(pos_robot.x - 0.5 );
            auxy = -(pos_robot.y - 0.3);
        }
        else if (pos_ball.x < -0.5)
        {
            auxx = -(pos_robot.x - -0.5 );
            auxy = -(pos_robot.y - 0.3);
        }
        else
        {
            auxx = -(pos_robot.x - pos_ball.x );
            auxy = -(pos_robot.y - 0.3);
        } 
    }
    else
    {
        if (pos_ball.x > 0.5)
        {
            auxx = -(pos_robot.x - 0.5 );
            auxy = -(pos_robot.y - -0.3);
        }
        else if (pos_ball.x < -0.5)
        {
            auxx = -(pos_robot.x - -0.5);
            auxy = -(pos_robot.y - -0.3);
        }
        else
        {
            auxx = -(pos_robot.x - pos_ball.x );
            auxy = -(pos_robot.y - -0.3);
        } 
    }
    ctrl::vec2 apf_vec = ctrl::vec2(auxx,auxy);
    apf_vec = ctrl::move_robot(robot, apf_vec, 0.392803, 40.0 * 0.822646 + 10.0);
    
    return 5*apf_vec;
}

