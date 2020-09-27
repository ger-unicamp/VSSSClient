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
    double apf_intensity = 4;

    ctrl::vec2 apf_vector = ctrl::vec2(apf_intensity, 0.0); // potential field vector at robot position
    
    return apf_vector;
} 

ctrl::vec2 apf::uniform_walls_field(fira_message::Robot &robot)
{
    double y = robot.y(); // force proportional to y and
    double x = -robot.x(); // inversely proportional to x coord of robot
    int y_sgn = ctrl::sgn(y);
    double ky = 5; // force strength in y axis
    double kx = 5; // force strength in x axis

    ctrl::vec2 apf_vector = ctrl::vec2(0, ky*y);
    
    return apf_vector;
}


ctrl::vec2 apf::robots_field(fira_message::Robot &robot1, fira_message::Robot &robot2)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double dist = sqrt(pow((robot2.x() - robot1.x()), 2) + pow((robot2.y() - robot1.y()), 2)); // distance to robot2
    double angle = atan2(-(robot2.y() - robot1.y()), (robot2.y() - robot1.x())); // angle to robot2
    double k = 0.20; // robot2 field strength scale

    apf_vector.x = -k*cos(angle)/pow(dist, 2);
    apf_vector.y = -k*sin(angle)/pow(dist, 2);
    // std::cout << "entre" << std::endl;
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 

ctrl::vec2 apf::ball_field(fira_message::Robot &robot, fira_message::Ball &ball)
{
    // hyperbolic spiral field
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double angle = atan2(-(ball.y() - robot.y()), (ball.x() - robot.x())); // angle to ball
    double dist = 3*sqrt(pow((ball.x() - robot.x()), 2) + pow((ball.y() - robot.y()), 2));
    double r = 2.0; // field center radius
    double s = 10.0; // field smoothing scale
    double k = 5.0; // field strenght scale
    double phi;
    
    // if (robot.x() < ball.x() + 0.2)
    // {
    //     apf_vector.x = dist*k*cos(angle);
    //     apf_vector.y = dist*k*sin(angle);
    // }
        
    // else
    {
        if (dist > r)
            phi = angle + (PI/2)*(2 - (r + s)/(dist + s));
        else if (dist >= 0 && dist <= r)
            phi = angle + (PI/2)*sqrt(dist/r);
        apf_vector.x = k*cos(phi);
        apf_vector.y = k*sin(phi);
    }

    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
}