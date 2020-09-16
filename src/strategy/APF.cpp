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
}

ctrl::vec2 apf::uniform_goal_field()
{
    double apf_intensity = 5;

    ctrl::vec2 apf_vector = ctrl::vec2(apf_intensity, 0.0); // potential field vector at robot position
    
    return apf_vector;
} 

ctrl::vec2 apf::uniform_walls_field(fira_message::Robot &robot)
{
    double y = robot.y(); // force proportional to y and
    double x = -robot.x(); // inversely proportional to x coord of robot
    double ky = 5; // force strength in y axis
    double kx = 2.5; // force strength in x axis

    ctrl::vec2 apf_vector = ctrl::vec2(kx*x, ky*y);
    
    return apf_vector;
}


ctrl::vec2 apf::robots_field(fira_message::Robot &robot1, fira_message::Robot &robot2)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double dist = sqrt(pow((robot2.x() - robot1.x()), 2) + pow((robot2.y() - robot1.y()), 2)); // distance to robot2
    double angle = atan2(-(robot2.y() - robot1.y()), (robot2.y() - robot1.x())); // angle to robot2
    double k = 0.1; // robot2 field strength scale

    apf_vector.x = -k*cos(angle)/pow(dist, 2);
    apf_vector.y = -k*sin(angle)/pow(dist, 2);
    // std::cout << "entre" << std::endl;
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 

ctrl::vec2 apf::ball_field(fira_message::Robot &robot, fira_message::Ball &ball)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double dist = sqrt(pow((ball.x() - robot.x()), 2) + pow((ball.x() - robot.y()), 2)); // distance to ball
    double angle = atan2(-(ball.y() - robot.y()), (ball.x() - robot.x())); // angle to ball
     
    double k = 15.0; // ball field strength scale

    apf_vector.x = k*cos(angle);
    apf_vector.y = k*sin(angle);
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 