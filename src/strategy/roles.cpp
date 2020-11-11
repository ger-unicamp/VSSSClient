#include "strategy/roles.h"
#include "goalkeeper.h"
#include "APF.h"
#include <iostream>

unsigned int robot_next_to_goal(vector<fira_message::Robot> my_robots, ctrl::vec2 target)
{
    double dist;
    unsigned int min_idx;
    double min_dist = std::numeric_limits<double>::max();
    for (auto robot : my_robots)
    {
        dist = ctrl::vec2(robot).distance(target);
        if (dist < min_dist) 
        {
            min_dist = dist;
            min_idx = robot.robot_id();
        }
    }

    return min_idx;
}

unsigned int robot_next_to_ball(vector<fira_message::Robot> my_robots, ctrl::vec2 ball)
{
    double dist;
    unsigned int min_idx = 4;
    double min_dist = std::numeric_limits<double>::max();
    for (auto robot : my_robots)
    {
        dist = ctrl::vec2(robot).distance(ball);
        if (dist < min_dist && dist < 2.0) 
        {
            min_dist = dist;
            min_idx = robot.robot_id();
        }
    }

    return min_idx;
}


ctrl::vec2 rol::goalkeeper(fira_message::Robot &robot, fira_message::Ball &ball)
{      
    ctrl::vec2 apf_vec;
    ctrl::vec2 future_ball = ctrl::future_position(ball,robot,DT);

    if (ctrl::vec2(robot).distance(future_ball) <= 0.077)
    {
        apf_vec = gpk::kick(robot, future_ball);
    }
    else
    {
        apf_vec = gpk::follow(robot, future_ball, K_LINE);
        apf_vec = ctrl::move_robot(robot,apf_vec, K_TURNING, K_VEL);
    }

    return apf_vec;
}

ctrl::vec2 rol::attacker(unsigned int moving_robot_id,fira_message::Ball &ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    ctrl::vec2 apf_vec;
    ctrl::vec2 future_ball = ctrl::future_position(ball,my_robots[moving_robot_id], DT);
    double spiral_phi = apf::move_to_goal(my_robots[moving_robot_id], future_ball, RADIUS, K_SPIRAL);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

ctrl::vec2 rol::defender(unsigned int moving_robot_id,fira_message::Ball &ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    ctrl::vec2 pos_ball = ctrl::vec2(ball);

    if (pos_ball.x < -0.4)
    {
        pos_ball.x = -0.4;
    }
    else if (pos_ball.x > 0.6)
    {
        pos_ball.x = 0.6;
    }

    pos_ball.y = 0.0;

    ctrl::vec2 apf_vec;
    double spiral_phi = apf::move_to_goal(my_robots[moving_robot_id], pos_ball, RADIUS, K_SPIRAL);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

vector<ctrl::vec2> rol::select_role(fira_message::Ball &ball,vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots)
{
    vector<ctrl::vec2> roles(3);
    ctrl::vec2 pos_ball = ctrl::future_position(ball, my_robots[0], DT);

    if (pos_ball.x <= 0)
    return roles;
}
