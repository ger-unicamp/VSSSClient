#include "strategy/APF.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

// moves robot in direction pointed by (x, y) vector
ctrl::vec2 apf::move_robot(fira_message::Robot &robot, ctrl::vec2 apf_vector)
{
    double vel = 4*apf_vector.abs();
    double angle = atan2(-apf_vector.y, apf_vector.x);
    double angle_diff = angle - robot.orientation();
    double k = 1; // turning gain constant

    double left_vel = vel*(cos(angle_diff) + k*sin(angle_diff));
    double right_vel = vel*(cos(angle_diff) - k*sin(angle_diff));
    ctrl::vec2 motors_vel = ctrl::vec2(right_vel, left_vel);

    return motors_vel;
}

ctrl::vec2 apf::goal_field(fira_message::Robot &robot)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double s = 5.0; // goal field spread
    double k = 20.0; // goal field strength scale
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

ctrl::vec2 apf::enemies_field(fira_message::Robot &robot, fira_message::Robot &enemy)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double s = 0.4; // enemy field spread
    double k = 40.0; // enemy field strength scale
    double r = 0.03; // enemy radius
    double dist = sqrt(pow((enemy.x() - robot.x()), 2) + pow((enemy.y() - robot.y()), 2)); // distance to enemy
    double angle = atan2(-(enemy.y() - robot.y()), (enemy.y() - robot.x())); // angle to enemy
     
    // std::cout << "dist" << dist << std::endl;

    if (dist < r) // if robot is colliding with enemy   
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

ctrl::vec2 apf::bros_field(fira_message::Robot &robot, fira_message::Robot &bro)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double s = 0.4; // bro field spread
    double k = 40.0; // bro field strength scale
    double r = 0.03; // bro radius
    double dist = sqrt(pow((bro.x() - robot.x()), 2) + pow((bro.y() - robot.y()), 2)); // distance to bro
    double angle = atan2(-(bro.y() - robot.y()), (bro.y() - robot.x())); // angle to bro
     
    // std::cout << "dist" << dist << std::endl;

    if (dist < r) // if robot is colliding with bro 
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
    double s = 3.0; // ball field spread
    double k = 20.0; // ball field strength scale
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