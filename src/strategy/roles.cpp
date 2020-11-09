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
        apf_vec = ctrl::move_robot(robot,apf_vec,K_TURNING, K_VEL);
    }

    return 6*apf_vec;
}

ctrl::vec2 rol::attacker(unsigned int moving_robot_id,fira_message::Ball &ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    ctrl::vec2 apf_vec;
    double spiral_phi = apf::move_to_goal(my_robots[moving_robot_id], ball, RADIUS, K_SPIRAL);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

ctrl::vec2 rol::defender(unsigned int moving_robot_id,fira_message::Ball &ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    ctrl::vec2 pos_ball = ctrl::vec2(ball);

    if (pos_ball.x < -0.5)
    {
        pos_ball.x = -0.5;
    }
    else if (pos_ball.x > 0.5)
    {
        pos_ball.x = 0.5;
    }

    if (moving_robot_id == 1)
    {
        pos_ball.y = 0.3;
    }
    else
    {
        pos_ball.y = -0.3;
    }

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
    ctrl::vec2 pos_ball = ctrl::vec2(ball);

    roles[0] = goalkeeper(my_robots[0], ball);

    if (pos_ball.y >= 0)
    {
        roles[1] = attacker(1,ball,my_robots,enemy_robots);
        roles[2] = defender(2,ball,my_robots,enemy_robots);
    }
    else
    {
        roles[1] = defender(1,ball,my_robots,enemy_robots);
        roles[2] = attacker(2,ball,my_robots,enemy_robots);
    }

    return roles;
}
