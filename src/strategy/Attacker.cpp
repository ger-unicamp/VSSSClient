#include "Attacker.h"

Attacker::Attacker(fira_message::Robot &robot): Player(robot) {}

/**
 * @brief get closest robot r from Attacker
 * 
 * @param my_robots 
 * @param enemy_robots 
 * @return fira_message::Robot 
 */
fira_message::Robot Attacker::get_closest_robot(std::vector<fira_message::Robot> robots)
{
    double min_dist = std::numeric_limits<double>::max();
    int idx = -1;
    for (uint i = 0; i < robots.size(); ++i)
    {
        double dist =this->get_pos().distance(this->future_position_of(robots[i], Player::DT));

        // get min dist of robot that isn't the same robot
        if (dist < min_dist && dist != 0) {
            min_dist = dist;
            idx = i;
        }
    }

    return robots[idx];
}

/**
 * @brief calculates final univector repulsion field created by obstacle
 * 
 * @param my_robots 
 * @param enemy_robots 
 * @return double
 */
double Attacker::univec_repulsion_field(fira_message::Robot obstacle)
{
    double phi = ctrl::vec2(this->get_pos() - obstacle).theta();

    return phi;
}

 /* @brief Calculates a spiral field arround (0, 0)
 * 
 * @param cw char indicating spiral orientation ('+' to counterclockwise and '-' to clockwise)
 * @return double 
 */
double Attacker::univec_spiral_field(ctrl::vec2 pos, char cw)
{
    double phi, dist, theta;
    double sgn = (cw == '-') ? -1.0 : 1.0;
    dist = pos.abs();
    theta = pos.theta();

    if (dist > Attacker::RADIUS)
        phi = theta + (sgn) * HALF_PI * (2.0 - ((Attacker::RADIUS + Attacker::K_SPIRAL) / (dist + Attacker::K_SPIRAL)));
    else
        phi = theta + (sgn) * HALF_PI * std::sqrt(dist / Attacker::RADIUS);

    return phi;
}

double Attacker::univec_spiral_field_cw(ctrl::vec2 pos)
{
    return univec_spiral_field(pos, '-');
}

double Attacker::univec_spiral_field_ccw(ctrl::vec2 pos)
{
    return univec_spiral_field(pos, '+');
}

/**
 * @brief combines both cw and ccw spiral fields to crate goal pointing vectors
 * 
 * @param target 
 * @return double 
 */
double Attacker::univec_spiral_field_to_target(ctrl::vec2 target)
{
    double phi;
    // univec always operate over translated points
    ctrl::vec2 translated = this->get_pos() - target;
    double yl = translated.y + Attacker::RADIUS;
    double yr = translated.y - Attacker::RADIUS;
    ctrl::vec2 posl = translated - ctrl::vec2(0, Attacker::RADIUS);
    ctrl::vec2 posr = translated + ctrl::vec2(0, Attacker::RADIUS);

    if (-Attacker::RADIUS <= translated.y && translated.y < Attacker::RADIUS)
    {
        double sin_phi, cos_phi;
        double phicw = this->univec_spiral_field_cw(posr);
        double phiccw = this->univec_spiral_field_ccw(posl);
        sincos(phicw, &sin_phi, &cos_phi); // Get sin & cosin at once
        ctrl::vec2 Ncw = ctrl::vec2(cos_phi, sin_phi);
        sincos(phiccw, &sin_phi, &cos_phi);
        ctrl::vec2 Nccw = ctrl::vec2(cos_phi, sin_phi);
        ctrl::vec2 tmp = (yl * Nccw - yr * Ncw) * (1.0 / (2.0 * Attacker::RADIUS));
        phi = tmp.theta();
    }
    else if (translated.y < -Attacker::RADIUS)
        phi = this->univec_spiral_field_cw(posl);
    else
        phi = this->univec_spiral_field_ccw(posr);

    return phi;
}

/**
 * @brief field that converge to target with final orientation in horizontal
 * 
 * @param target 
 * @return double 
 */
double Attacker::univec_horizontal_line_field(ctrl::vec2 target)
{
    double phi;
    // Univector always operate over translated points
    ctrl::vec2 translated = this->get_pos() - target;
    double e = std::exp(Player::K_LINE * translated.y);

    if (translated.x <= 0.0)
        phi = HALF_PI * ((1 - e) / (1 + e));
    else
        phi = HALF_PI * ((1 + 3 * e) / (1 + e));

    return phi;
}

/**
 * @brief composes repulsion and move to goal fields with gaussian compound ratio
 * 
 * @param repulsion_phi 
 * @param spiral_phi
 * @return double final composed phi
 */
double Attacker::univec_composite_field(double repulsion_phi, double spiral_phi)
{
    double gauss = math::gaussian(Attacker::RADIUS - Attacker::D_MIN, Attacker::SIGMA);
    if (Attacker::RADIUS <= Attacker::D_MIN)
        return repulsion_phi;
    else
        return (repulsion_phi * gauss + spiral_phi * (1-gauss));
}