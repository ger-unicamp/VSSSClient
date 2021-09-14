#include "Attacker.h"
#include "strategy/Game.h"

Attacker::Attacker(fira_message::Robot &robot): Player(robot) {}

 /** @brief Calculates a spiral field arround (0, 0)
 * 
 * @param cw char indicating spiral orientation ('+' to counterclockwise and '-' to clockwise)
 * @return double 
 */
double Attacker::univec_spiral_field(ctrl::vec2 pos, bool is_cw)
{
    double phi, dist, theta;
    double sgn = (is_cw) ? -1.0 : 1.0;
    dist = pos.abs();
    theta = pos.theta();

    if (dist > Attacker::RADIUS)
        phi = theta + (sgn) * HALF_PI * (2.0 - ((Attacker::RADIUS + Attacker::K_SPIRAL) / (dist + Attacker::K_SPIRAL)));
    else
        phi = theta + (sgn) * HALF_PI * std::sqrt(dist / Attacker::RADIUS);

    return phi;
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
        double phicw = this->univec_spiral_field(posr, Attacker::CW);
        double phiccw = this->univec_spiral_field(posl, Attacker::CCW);
        sincos(phicw, &sin_phi, &cos_phi); // Get sin & cosin at once
        ctrl::vec2 Ncw = ctrl::vec2(cos_phi, sin_phi);
        sincos(phiccw, &sin_phi, &cos_phi);
        ctrl::vec2 Nccw = ctrl::vec2(cos_phi, sin_phi);
        ctrl::vec2 tmp = (yl * Nccw - yr * Ncw) * (1.0 / (2.0 * Attacker::RADIUS));
        phi = tmp.theta();
    }
    else if (translated.y < -Attacker::RADIUS)
        phi = this->univec_spiral_field(posl, Attacker::CW);
    else
        phi = this->univec_spiral_field(posr, Attacker::CCW);

    return phi;
}

/**
 * @brief field that converge to target with final orientation in horizontal
 * 
 * @param target 
 * @return double 
 */
double Attacker::univec_horizontal_sigmoid_field(ctrl::vec2 target)
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
 * @brief rotates the spiral field directing the ball to the center of the enemy goal
 * 
 * 
 * @param phi 
 * @return double 
 */
double Attacker::univec_rotate(double phi)
{
    ctrl::vec2 ball_future_position = Game::get_ball_future_position(this->DT);
    double theta = atan2(-ball_future_position.y, Game::FIELD_LIMIT_X - ball_future_position.x);

    return phi + theta;
}

/**
 * @brief Attacker behaviour - returns attacker final motors speed
 * 
 * @param robots 
 * @return ctrl::vec2 
 */
ctrl::vec2 Attacker::behaviour(std::vector<fira_message::Robot> &robots){
    fira_message::Robot closest_robot;
    ctrl::vec2 univec, motors_speeds, ball_fut_pos;
    double spiral_phi, repulsion_phi, phi;

    ball_fut_pos = this->future_position_relative_to_ball();

    if (this->get_pos().x < -Game::ATTACKER_LINE_X)
    {
        ball_fut_pos.x = math::bound(ball_fut_pos.x, -Game::ATTACKER_LINE_X, Player::INF);
        spiral_phi = this->univec_horizontal_sigmoid_field(ball_fut_pos);
    }

    else
    {
        spiral_phi = this->univec_spiral_field_to_target(ball_fut_pos);
        spiral_phi = this->univec_rotate(spiral_phi);
    }
    
    closest_robot = this->get_closest_robot(robots);
    repulsion_phi = this->univec_repulsion_field(closest_robot);
    phi = this->univec_composite_field(repulsion_phi, spiral_phi, this->get_pos().distance(closest_robot));
    sincos(phi, &univec.y, &univec.x);

    motors_speeds = this->move(univec);

    return motors_speeds;
}