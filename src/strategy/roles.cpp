#include "roles.h"
#include "goalkeeper.h"
#include "APF.h"
#include <iostream>

extern unsigned int stopped_count_gkp;
extern unsigned int stopped_count_atk;
extern unsigned int stopped_count_def;

unsigned int rol::robot_next_to_target(unsigned int id_gkp, unsigned int last_idx, vector<fira_message::Robot> my_robots, fira_message::Ball target)
{
    const double epsilon = 0.1;
    double dist; 
    unsigned int min_idx = last_idx;
    double min_dist = ctrl::vec2(my_robots[last_idx]).distance(target);
    ctrl::vec2 future_pos_robot, future_pos_target;

    for (auto robot : my_robots)
    {   
        ctrl::vec2 vel(robot.vx(), robot.vy());
        future_pos_robot = ctrl::vec2(robot) + DT * vel;
        future_pos_target = ctrl::future_position(target, robot, DT);
        dist = future_pos_robot.distance(future_pos_target);
        if (dist < min_dist - epsilon && robot.robot_id() != id_gkp) 
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
    ctrl::vec2 future_ball = ctrl::future_position(ball, robot, DT_GOALKEEPER);

    if (ctrl::vec2(robot).distance(future_ball) <= 0.077)
    {
        apf_vec = gpk::kick((robot.y() < future_ball.y));
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
    double move_phi, phi;

    if (ctrl::vec2(my_robots[moving_robot_id]).x < -0.55)
    {
        future_ball.x = math::bound(future_ball.x, -0.55, std::numeric_limits<double>::max());
        move_phi = apf::horizontal_line_field(my_robots[moving_robot_id], future_ball, K_LINE);
    }

    else
    {
        move_phi = apf::move_to_goal(my_robots[moving_robot_id], future_ball, RADIUS, K_SPIRAL);
    }
    
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    phi = apf::composite_field(tmp.first, move_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

ctrl::vec2 rol::defender(unsigned int moving_robot_id, ctrl::vec2 pos_target, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots)
{
    pos_target.y = math::bound(pos_target.y, -0.4, 0.4);
    if (pos_target.x >= 0)
    { 
        pos_target.x = math::bound(pos_target.x - 0.4, -0.4, 0.4);
    }
    else 
    {
        pos_target.x = math::bound(pos_target.x + 0.4, -0.4, 0.4);
    }
    ctrl::vec2 apf_vec;
    double spiral_phi = apf::vertical_line_field(my_robots[moving_robot_id], pos_target, K_LINE);
    pair<double,double> tmp = apf::repulsion_field(moving_robot_id, my_robots, enemy_robots, DT);
    double phi = apf::composite_field(tmp.first, spiral_phi, SIGMA, D_MIN, tmp.second);
    sincos(phi,&apf_vec.y,&apf_vec.x);
    apf_vec = ctrl::move_robot(my_robots[moving_robot_id], apf_vec, K_TURNING, K_VEL);

    return apf_vec;
}

bool rol::liberty_spin(fira_message::Robot robot, unsigned int &stopped_count)
{
    ctrl::vec2 robot_vel(robot.vx(), robot.vy());

    if (robot_vel.abs() > 0.1 || stopped_count >= 70) stopped_count = 0;
    else ++stopped_count;

    return (stopped_count >= 60);
}

vector<ctrl::vec2> rol::select_role(fira_message::Ball &ball,vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots)
{
    const unsigned int id_gkp = 0;
    static unsigned int id_atk = 1, id_def = 2;

    vector<ctrl::vec2> roles(3);

    id_atk = robot_next_to_target(id_gkp, id_atk, my_robots, ball);
    id_def = 3 - (id_gkp + id_atk);

 
    if (liberty_spin(my_robots[id_gkp], stopped_count_gkp))
    {
        roles[id_gkp] = gpk::kick((my_robots[id_gkp].y() < ball.y()));
    } 
    else
    {
        roles[id_gkp] = goalkeeper(my_robots[id_gkp], ball);
    }
    

    if (liberty_spin(my_robots[id_atk], stopped_count_atk))
    {
        roles[id_atk] = gpk::kick((my_robots[id_atk].y() > 0.0));
    }  
    else
    {
        roles[id_atk] = attacker(id_atk, ball, my_robots, enemy_robots);
    }

     
    if (liberty_spin(my_robots[id_def], stopped_count_def))
    {
        roles[id_def] = gpk::kick((my_robots[id_def].y() > 0.0));
    } 
    else
    {
        roles[id_def] = defender(id_def, ball, my_robots, enemy_robots);
    }
    

    std::cout << "id atacante: " << id_atk << endl;

    return roles;
}