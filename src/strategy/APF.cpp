#include "strategy/APF.h"
#include <iostream>

#define GOAL_X 0.75
#define GOAL_Y 0.0

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
 * @brief composses univector repulsion fields by all obstacles
 * 
 * @param moving_robot_id 
 * @param my_robots 
 * @param enemy_robots 
 * @return std::pair<double, double> returns min distance phi and min distance
 */
std::pair<double, double> apf::repulsion_field(unsigned int moving_robot_id, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots, double dt)
{
    double repulsion_phi;
    std::vector<double> phis;
    double dist;
    std::vector<double> distances;
    ctrl::vec2 obstacle;

    for (size_t i = 0; i < my_robots.size(); ++i)
    {
        if (i != moving_robot_id)
        {
            obstacle =  ctrl::future_position(my_robots[i], my_robots[moving_robot_id], dt);
            repulsion_phi = (ctrl::vec2(my_robots[moving_robot_id]) - obstacle).theta();
            phis.push_back(repulsion_phi);
            dist = ctrl::vec2(my_robots[moving_robot_id]).distance(obstacle);
            distances.push_back(dist);
        }
    }
    for (size_t i = 0; i < enemy_robots.size(); ++i)
    {
        obstacle =  ctrl::future_position(enemy_robots[i], my_robots[moving_robot_id], dt);
        repulsion_phi = (ctrl::vec2(my_robots[moving_robot_id]) - obstacle).theta();
        phis.push_back(repulsion_phi);
        dist = ctrl::vec2(my_robots[moving_robot_id]).distance(obstacle);
        distances.push_back(dist);
    }

    auto min_dist_it = std::min_element(distances.begin(), distances.end());
    return std::make_pair(phis[min_dist_it - distances.begin()], *min_dist_it);
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

/**
 * @brief composes repulsion and move to goal fields with gaussian compound ratio
 * 
 * @param repulsion_vec 
 * @param spiral_vec 
 * @param sigma 
 * @param dmin 
 * @param R 
 * @return double 
 */
double apf::composite_field(double repulsion_phi, double spiral_phi, double sigma, double dmin, double R)
{
    double gauss = math::gaussian(R-dmin, sigma);
    if (R <= dmin)
        return repulsion_phi;
    else
        return (repulsion_phi * gauss + spiral_phi * (1-gauss));
}