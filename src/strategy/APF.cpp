#include "strategy/APF.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

// moves robot in direction pointed by (x, y) vector
ctrl::vec2 apf::move_robot(fira_message::Robot &robot, ctrl::vec2 apf_vector, double k, double v)
{
    // k is the turning gain constant and v is the velocity constant
    double left_vel, right_vel;
    double vel = v*apf_vector.abs();
    double angle = atan2(-apf_vector.y, apf_vector.x);
    double angle_diff = angle - robot.orientation();  
    if (angle_diff >= PI/2 || angle_diff <= -PI/2)
    {
        right_vel = vel*(cos(angle_diff) + k*sin(angle_diff));
        left_vel = vel*(cos(angle_diff) - k*sin(angle_diff));
    } 
    else
    {
        left_vel = vel*(cos(angle_diff) + k*sin(angle_diff));
        right_vel = vel*(cos(angle_diff) - k*sin(angle_diff));
    }
    
    ctrl::vec2 motors_vel = ctrl::vec2(right_vel, left_vel);
    return motors_vel;
}

ctrl::vec2 apf::uniform_goal_field()
{
    double apf_intensity = 10;

    ctrl::vec2 apf_vector = ctrl::vec2(apf_intensity, 0.0); // potential field vector at robot position
    
    return apf_vector;
} 

ctrl::vec2 apf::uniform_walls_field(fira_message::Robot &robot)
{
    double y = robot.y(); // force proportional to y and
    double x = -robot.x(); // inversely proportional to x coord of robot
    int y_sgn = ctrl::sgn(y);
    double ky = 5; // force strength in y axis
    double kx = 2; // force strength in x axis

    ctrl::vec2 apf_vector = ctrl::vec2(x>0?-kx:kx, y>0?ky:-ky);
    
    return apf_vector;
}


ctrl::vec2 apf::robots_field(fira_message::Robot &robot1, fira_message::Robot &robot2)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0); // potential field vector at robot position
    double dist = sqrt(pow((robot2.x() - robot1.x()), 2) + pow((robot2.y() - robot1.y()), 2)); // distance to robot2
    double angle = atan2(-(robot2.y() - robot1.y()), (robot2.x() - robot1.x())); // angle to robot2
    double k = 0.20; // robot2 field strength scale

    apf_vector.x = -k*cos(angle)/pow(dist, 2);
    apf_vector.y = -k*sin(angle)/pow(dist, 2);
    // std::cout << "entre" << std::endl;
    
    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
} 


//  ----------> BALL FIELD <-----------

double apf::spiral_field_cw(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k)
{
    double phi, dist, theta;
    dist = sqrt(pow((pos.x - target.x), 2) + pow((pos.y - target.y), 2));
    theta = pos.theta();

    if (dist > radius)
        phi = theta - (PI/2) * (2 - radius+k) / (dist+k);
    else
        phi = theta - (PI/2) * sqrt(dist/radius);

    return phi;
}

double apf::spiral_field_ccw(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k)
{
    double phi, dist, theta;
    dist = sqrt(pow((pos.x - target.x), 2) + pow((pos.y - target.y), 2));
    theta = pos.theta();
    
    if (dist > radius)
        phi = theta + (PI/2) * (2 - radius+k) / (dist+k);
    else
        phi = theta + (PI/2) * sqrt(dist/radius);

    return phi;
}

double apf::move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k)
{
    double phi;
    double yl = pos.y + radius;
    double yr = pos.y - radius;
    ctrl::vec2 posl = ctrl::vec2(pos.x, yr);
    ctrl::vec2 posr = ctrl::vec2(pos.x, yl);
    double phicw = apf::spiral_field_cw(posl, target, radius, k);
    double phiccw = apf::spiral_field_ccw(posr, target, radius, k);
    ctrl::vec2 Ncw = ctrl::vec2(cos(phicw), sin(phicw));
    ctrl::vec2 Nccw = ctrl::vec2(cos(phiccw), sin(phiccw));

    if ((-radius + target.y) <= pos.y  && pos.y < (radius + target.y))
    {
        ctrl::vec2 tmp = (yl*Nccw + yr*Ncw) * (1/(2*radius));
        phi = 0;
    }

    else if (pos.y < -radius + target.y)
        phi = apf::spiral_field_cw(posl, target, radius, k);

    else if (pos.y >= radius + target.y)
        phi = apf::spiral_field_ccw(posr, target, radius, k);
    
    return phi;
}


ctrl::vec2 apf::ball_field(fira_message::Robot &robot, fira_message::Ball &ball, double radius, double k)
{
    ctrl::vec2 pos = ctrl::vec2(robot.x(), robot.y());
    ctrl::vec2 target = ctrl::vec2(ball.x(), ball.y());

    double phi = move_to_goal(pos, target, radius, k);

    ctrl::vec2 apf_vector = ctrl::vec2(cos(phi), sin(phi));

    std::cout << "vec theta: " << apf_vector.theta()*180/PI << std::endl;

    return apf_vector;
}

ctrl::vec2 apf::test_control(fira_message::Robot &robot, fira_message::Ball &ball)
{
    double angle = atan2(-(ball.y() - robot.y()), ball.x() - robot.x()); 
    ctrl::vec2 apf = ctrl::vec2(cos(angle), sin(angle));
    return apf;
}