#include "strategy/roles.h"
#include "goalkeeper.h"
#include "APF.h"
#include <iostream>

const unsigned int NULL_ID = 4;

unsigned int rol::robot_next_to_target(unsigned int id_gkp, vector<fira_message::Robot> my_robots, ctrl::vec2 target)
{
    double dist;
    unsigned int min_idx = NULL_ID;
    double min_dist = std::numeric_limits<double>::max();
    for (auto robot : my_robots)
    {
        dist = ctrl::vec2(robot).distance(target);
        if (dist < min_dist && robot.robot_id() != id_gkp) 
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
    if (future_ball.x < -0.4)
    {
        future_ball.x = -0.4;
    }
    double spiral_phi = apf::move_to_goal(my_robots[moving_robot_id], future_ball, RADIUS, K_SPIRAL);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

ctrl::vec2 rol::defender(unsigned int moving_robot_id, ctrl::vec2 pos_ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    if (pos_ball.x < -0.4)
    {
        pos_ball.x = -0.4;
    }
    else if (pos_ball.x > 0.4)
    {
        pos_ball.x = 0.4;
    }

    pos_ball.y = 0.0;

    ctrl::vec2 apf_vec;
    // double spiral_phi = apf::move_to_goal(my_robots[moving_robot_id], pos_ball, RADIUS, K_SPIRAL);
    double spiral_phi = apf::horizontal_line_field(my_robots[moving_robot_id], pos_ball, K_LINE);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

ctrl::vec2 rol::defender_midfield(unsigned int moving_robot_id, ctrl::vec2 pos_ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    if (pos_ball.x >= 0) 
    {
        pos_ball.x = pos_ball.x - 0.2;
        pos_ball.y = (pos_ball.y > 0.4)? 0.4 : pos_ball.y;
        pos_ball.y = (pos_ball.y < -0.4)? -0.4 : pos_ball.y;
    }
    else pos_ball.x = -0.2;


    ctrl::vec2 apf_vec;
    // double spiral_phi = apf::move_to_goal(my_robots[moving_robot_id], pos_ball, RADIUS, K_SPIRAL);
    double spiral_phi = apf::vertical_line_field(my_robots[moving_robot_id], pos_ball, K_LINE);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

vector<ctrl::vec2> rol::select_role(fira_message::Ball &ball,vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots)
{
    unsigned int id_gkp, id_atk, id_def;
    vector<ctrl::vec2> roles(3);
    ctrl::vec2 pos_ball = ctrl::future_position(ball, my_robots[0], DT);
    ctrl::vec2 goal = {-0.65, 0.0};

    id_gkp = 0;
    id_atk = robot_next_to_target(id_gkp, my_robots, pos_ball);
    id_def = 3 - (id_gkp + id_atk);
    roles[id_gkp] = goalkeeper(my_robots[id_gkp], ball);
    roles[id_atk] = attacker(id_atk, ball, my_robots, enemy_robots);
    roles[id_def] = defender_midfield(id_def, pos_ball, my_robots, enemy_robots);


    return roles;
}
