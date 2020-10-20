#include "strategy/APF.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

// moves robot in direction pointed by (x, y) vector
ctrl::vec2 apf::move_robot(fira_message::Robot &robot, ctrl::vec2 apf_vector, double k, double v)
{
    // k is the turning gain constant and v is the velocity constant
    double left_vel, right_vel;
    double vel = v * apf_vector.abs();
    double angle = atan2(-apf_vector.y, apf_vector.x);
    double angle_diff = angle - robot.orientation();
    if (angle_diff >= PI / 2 || angle_diff <= -PI / 2)
    {
        right_vel = vel * (cos(angle_diff) + k * sin(angle_diff));
        left_vel = vel * (cos(angle_diff) - k * sin(angle_diff));
    }
    else
    {
        left_vel = vel * (cos(angle_diff) + k * sin(angle_diff));
        right_vel = vel * (cos(angle_diff) - k * sin(angle_diff));
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
    double y = robot.y();  // force proportional to y and
    double x = -robot.x(); // inversely proportional to x coord of robot
    int y_sgn = ctrl::sgn(y);
    double ky = 5; // force strength in y axis
    double kx = 2; // force strength in x axis

    ctrl::vec2 apf_vector = ctrl::vec2(x > 0 ? -kx : kx, y > 0 ? ky : -ky);

    return apf_vector;
}

ctrl::vec2 apf::robots_field(fira_message::Robot &robot1, fira_message::Robot &robot2)
{
    ctrl::vec2 apf_vector = ctrl::vec2(0.0);                                                   // potential field vector at robot position
    double dist = sqrt(pow((robot2.x() - robot1.x()), 2) + pow((robot2.y() - robot1.y()), 2)); // distance to robot2
    double angle = atan2(-(robot2.y() - robot1.y()), (robot2.x() - robot1.x()));               // angle to robot2
    double k = 0.20;                                                                           // robot2 field strength scale

    apf_vector.x = -k * cos(angle) / pow(dist, 2);
    apf_vector.y = -k * sin(angle) / pow(dist, 2);
    // std::cout << "entre" << std::endl;

    // std::cout << apf_vector.x << apf_vector.y << std::endl;
    return apf_vector;
}

//  ----------> BALL FIELD <-----------
/**
 * @brief Calculates a spiral field arround (0, 0)
 * 
 * @param pos 
 * @param radius 
 * @param k 
 * @param cw char indicating spiral orientation ('+' to clockwise and '-' to counterclockwise)
 * @return double 
 */
double spiral_field(ctrl::vec2 pos, double radius, double k, char cw)
{
    double phi, dist, theta;
    double sgn = (cw == '-') ? -1.0 : 1.0;
    dist = pos.abs();
    theta = pos.theta();

    if (dist > radius)
        phi = theta + (sgn) * (PI / 2.0) * (2.0 - ((radius + k) / (dist + k)));
    else
        phi = theta + (sgn) * (PI / 2.0) * std::sqrt(dist / radius);

    return phi;
}

double apf::spiral_field_cw(ctrl::vec2 pos, double radius, double k)
{
    return spiral_field(pos, radius, k, '+');
}

double apf::spiral_field_ccw(ctrl::vec2 pos, double radius, double k)
{
    return spiral_field(pos, radius, k, '-');
}

double apf::move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k)
{
    double phi;
    // Univector always operate over translated points
    ctrl::vec2 translated = pos - target;
    double yl = translated.y + radius;
    double yr = translated.y - radius;
    ctrl::vec2 posl = translated - ctrl::vec2(0, radius);
    ctrl::vec2 posr = translated + ctrl::vec2(0, radius);

    if (-radius <= translated.y && translated.y < radius)
    {
        double phicw = apf::spiral_field_cw(posr, radius, k);
        double phiccw = apf::spiral_field_ccw(posl, radius, k);
        ctrl::vec2 Ncw = ctrl::vec2(cos(phicw), sin(phicw));
        ctrl::vec2 Nccw = ctrl::vec2(cos(phiccw), sin(phiccw));
        ctrl::vec2 tmp = (yl * Nccw + yr * Ncw) * (1.0 / (2.0 * radius));
        phi = tmp.theta();
    }
    else if (translated.y < -radius)
        phi = apf::spiral_field_cw(posl, radius, k);
    else
        phi = apf::spiral_field_ccw(posr, radius, k);

    return phi;
}

ctrl::vec2 apf::ball_field(fira_message::Robot &robot, fira_message::Ball &ball, double radius, double k)
{
    ctrl::vec2 pos = ctrl::vec2(robot);
    ctrl::vec2 target = ctrl::vec2(ball);

    double phi = move_to_goal(pos, target, radius, k);

    ctrl::vec2 apf_vector = ctrl::vec2(cos(phi), sin(phi));

    std::cout << "vec theta: " << apf_vector.theta() * 180 / PI << std::endl;

    return apf_vector;
}

ctrl::vec2 apf::test_control(fira_message::Robot &robot, fira_message::Ball &ball)
{
    double angle = atan2(-(ball.y() - robot.y()), ball.x() - robot.x());
    ctrl::vec2 apf = ctrl::vec2(cos(angle), sin(angle));
    return apf;
}