#include "strategy/APF.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

// moves robot in direction pointed by (x, y) vector
ctrl::vec2 apf::move_robot(fira_message::Robot &robot, ctrl::vec2 apf_vector)
{
    double vel = 3*apf_vector.abs();
    double angle = atan2(-apf_vector.y, apf_vector.x);
    double angle_diff = angle - robot.orientation();
    double k = 1; // turning gain constant
    double left_vel = vel*(cos(angle_diff) + k*sin(angle_diff));
    double right_vel = vel*(cos(angle_diff) - k*sin(angle_diff));
    ctrl::vec2 motors_vel = ctrl::vec2(right_vel, left_vel);
    return motors_vel;

    // double kv = 2*apf_vector.abs();
    // double set_angle = apf_vector.theta() - robot.orientation();
    // double kt = 40.0, eps = 0.1;

    // if (set_angle < PI / 2.0 + eps && set_angle > PI / 2.0 - eps)
    // {
    //     set_angle = PI / 2.0;
    //     kv = 0.0;
    // }
    // else if (set_angle < -PI / 2.0 + eps && set_angle > -PI / 2.0 - eps)
    // {
    //     set_angle = -PI / 2.0;
    //     kv = 0.0;
    // }
    // else if (set_angle > PI / 2.0)
    // {
    //     set_angle -= PI;
    //     kv *= -1.0;
    // }
    // else if (set_angle < -PI / 2.0)
    // {
    //     set_angle += PI;
    //     kv *= -1.0;
    // }
    // std::cout << set_angle << std::endl;
    // return kt * ctrl::vec2(-set_angle, set_angle) + kv * ctrl::vec2(1);

}

ctrl::vec2 apf::enemy_goal_field(fira_message::Robot &robot)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double s = 5.0; // goal field spread
    double k = 30.0; // goal field strength scale
    double r = 0.05; // goal radius
    double dist = sqrt(pow((GOAL_X - robot.x()), 2) + pow((GOAL_Y - robot.y()), 2)); // distance to goal
    double angle = atan2(-(GOAL_Y - robot.y()), (GOAL_X - robot.x())); // angle to goal
     
    // std::cout << "dist" << dist << std::endl;

    if (dist < r) // if robot is inside the goal
    {
        apf_vector = ctrl::vec2(0.0);
        // std::cout << "dentro" << std::endl;
    }
    else if (r <= dist && dist <= s) // if robot is inside field spread
    {
        apf_vector.x = k*(dist-r)*cos(angle);
        apf_vector.y = k*(dist-r)*sin(angle);
        // std::cout << "entre" << std::endl;
    }

    else if (dist > s + r) // if robot is outside field spread
    {
        apf_vector.x = k*s*cos(angle);
        apf_vector.y = k*s*sin(angle);
        // std::cout << "fora" << std::endl;    
    }
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 


ctrl::vec2 apf::robots_field(fira_message::Robot &robot1, fira_message::Robot &robot2)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double s = 0.5; // robot2 field spread
    double k = 30.0; // robot2 field strength scale
    double r = 0.03; //  radius
    double dist = sqrt(pow((robot2.x() - robot1.x()), 2) + pow((robot2.y() - robot1.y()), 2)); // distance to robot2
    double angle = atan2(-(robot2.y() - robot1.y()), (robot2.y() - robot1.x())); // angle to robot2
     
    // std::cout << "dist" << dist << std::endl;

    if (dist < r) // if robot is colliding with robot2   
    {
        apf_vector.x = -ctrl::sgn(cos(angle))*20.0;
        apf_vector.y = -ctrl::sgn(sin(angle))*20.0;
        // std::cout << "dentro" << std::endl;
    }
    else if (r <= dist && dist <= s) // if robot is inside field spread
    {
        apf_vector.x = -k*(s + r - dist)*cos(angle);
        apf_vector.y = -k*(s + r - dist)*sin(angle);
        // std::cout << "entre" << std::endl;
    }

    else if (dist > s + r) // if robot is outside field spread
    {
        apf_vector = ctrl::vec2(0.0);
        // std::cout << "fora" << std::endl;    
    }
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 

ctrl::vec2 apf::ball_field(fira_message::Robot &robot, fira_message::Ball &ball)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double s = 2.0; // ball field spread
    double k = 50.0; // ball field strength scale
    double r = 0.005; // ball radius
    double dist = sqrt(pow((ball.x() - robot.x()), 2) + pow((ball.x() - robot.y()), 2)); // distance to ball
    double angle = atan2(-(ball.y() - robot.y()), (ball.x() - robot.x())); // angle to ball
     
    std::cout << "dist" << dist << std::endl;

    if (dist < r) // if robot is inside the ball
    {
        apf_vector = ctrl::vec2(0.0);
        std::cout << "dentro" << std::endl;
    }
    else if (r <= dist && dist <= s) // if robot is inside field spread
    {
        apf_vector.x = k*(dist-r)*cos(angle);
        apf_vector.y = k*(dist-r)*sin(angle);
        std::cout << "entre" << std::endl;
    }

    else if (dist > s + r) // if robot is outside field spread
    {
        apf_vector.x = k*s*cos(angle);
        apf_vector.y = k*s*sin(angle);
        std::cout << "fora" << std::endl;    
    }
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 