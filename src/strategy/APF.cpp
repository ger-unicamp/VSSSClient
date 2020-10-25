#include "strategy/APF.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

ctrl::vec2 apf::uniform_goal_field()
{
    double apf_intensity = 10;

    ctrl::vec2 apf_vector = ctrl::vec2(apf_intensity, 0.0); // potential field vector at robot position

    return apf_vector;
}

ctrl::vec2 apf::uniform_walls_field(ctrl::vec2 robot)
{
    double y = robot.y;  // force proportional to y and
    double x = -robot.x; // inversely proportional to x coord of robot
    double ky = 5;         // force strength in y axis
    double kx = 2;         // force strength in x axis

    ctrl::vec2 apf_vector = ctrl::vec2(x > 0 ? -kx : kx, y > 0 ? ky : -ky);

    return apf_vector;
}

/**
 * @brief Returns robot2 repulsive field for robot1
 * 
 * @param robot1 
 * @param robot2 
 * @param k robot2 field strength scale
 * @return ctrl::vec2 
 */
ctrl::vec2 apf::robots_field(ctrl::vec2 robot1, ctrl::vec2 robot2, double k)
{
    ctrl::vec2 apf_vector; // potential field vector at robot position
    ctrl::vec2 tr = robot2 - robot1;
    double dist = tr.abs();    // distance to robot2
    double angle = tr.theta(); // angle to robot

    apf_vector = (-k / (dist * dist)) * ctrl::vec2(cos(angle), sin(angle));
    return apf_vector;
}

//  ----------> BALL FIELD <-----------
/**
 * @brief Calculates a spiral field arround (0, 0)
 * 
 * @param pos 
 * @param radius 
 * @param k 
 * @param cw char indicating spiral orientation ('+' to counterclockwise and '-' to clockwise)
 * @return double 
 */
double spiral_field(ctrl::vec2 pos, double radius, double k, char cw)
{
    double phi, dist, theta;
    double sgn = (cw == '-') ? -1.0 : 1.0;
    dist = pos.abs();
    theta = pos.theta();

    if (dist > radius)
        phi = theta + (sgn) * HALF_PI * (2.0 - ((radius + k) / (dist + k)));
    else
        phi = theta + (sgn) * HALF_PI * std::sqrt(dist / radius);

    return phi;
}

double apf::spiral_field_cw(ctrl::vec2 pos, double radius, double k)
{
    return spiral_field(pos, radius, k, '-');
}

double apf::spiral_field_ccw(ctrl::vec2 pos, double radius, double k)
{
    return spiral_field(pos, radius, k, '+');
}

/**
 * @brief 
 * 
 * @param pos 
 * @param target 
 * @param radius 
 * @param k 
 * @return double 
 */
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
        double sin_phi, cos_phi;
        double phicw = apf::spiral_field_cw(posr, radius, k);
        double phiccw = apf::spiral_field_ccw(posl, radius, k);
        sincos(phicw, &sin_phi, &cos_phi); // Get sin & cosin at once
        ctrl::vec2 Ncw = ctrl::vec2(cos_phi, sin_phi);
        sincos(phiccw, &sin_phi, &cos_phi);
        ctrl::vec2 Nccw = ctrl::vec2(cos_phi, sin_phi);
        ctrl::vec2 tmp = (yl * Nccw - yr * Ncw) * (1.0 / (2.0 * radius));
        phi = tmp.theta();
    }
    else if (translated.y < -radius)
        phi = apf::spiral_field_cw(posl, radius, k);
    else
        phi = apf::spiral_field_ccw(posr, radius, k);

    return phi;
}

ctrl::vec2 apf::ball_field(ctrl::vec2 robot, ctrl::vec2 ball, double radius, double k)
{
    double phi = move_to_goal(robot, ball, radius, k);
    double sin_phi, cos_phi;
    sincos(phi, &sin_phi, &cos_phi);
    ctrl::vec2 apf_vector = ctrl::vec2(cos_phi, sin_phi);

    std::cout << "vec theta: " << apf_vector.theta() * 180 / PI << std::endl;

    return apf_vector;
}

ctrl::vec2 apf::test_control(ctrl::vec2 robot, ctrl::vec2 ball)
{
    double angle = (ball - robot).theta();
    ctrl::vec2 apf = ctrl::vec2(cos(angle), sin(angle));
    return apf;
}