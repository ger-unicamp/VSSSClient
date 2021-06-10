#include "Goalkeeper.h"

Goalkeeper::Goalkeeper(fira_message::Robot &robot): Player(robot) {}

/**
 * @brief field that converge to target with final orientation in vertical
 * 
 * @param target 
 * @return double 
 */
double Goalkeeper::univec_vertical_line_field(ctrl::vec2 target)
{
    double phi;

    // univector always operate over translated points
    ctrl::vec2 translated = ctrl::vec2(this->robot) - target;

    if (translated.y <= 0.0)
        phi = PI * (1.0 / (1.0 + std::exp(-1.0 * Goalkeeper::K_LINE * translated.x)));
    else
        phi = PI * (-1.0 / (1.0 + std::exp(-1.0 * Goalkeeper::K_LINE * translated.x)));

    return phi;
}

/**
 * @brief defend our goal by following ball y projection using a vertical line univector field
 * 
 * @param pos_ball 
 * @return ctrl::vec2 
 */
ctrl::vec2 Goalkeeper::follow_ball_projection(ctrl::vec2 pos_ball)
{
    const double x_target = Goalkeeper::X_LIMIT_GKP;
    double y_target;

    // if ball is too close, goalkeeper can get out of y limit
    if (pos_ball.x < Goalkeeper::X_LIMIT_BALL)
        y_target = pos_ball.y;
    else
        y_target = math::bound(pos_ball.y, -Goalkeeper::Y_LIMIT_GKP, Goalkeeper::Y_LIMIT_GKP);
    
    ctrl::vec2 target = {std::min(x_target, pos_ball.x), y_target};

    double phi = Goalkeeper::univec_vertical_line_field(target);
    ctrl::vec2 apf_vec;
    sincos(phi, &apf_vec.y, &apf_vec.x);

    return apf_vec;
}
