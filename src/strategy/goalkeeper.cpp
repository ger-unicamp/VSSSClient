#include "strategy/goalkeeper.h"
#include <iostream>


ctrl::vec2 gpk::follow(ctrl::vec2 pos_robot, ctrl::vec2 pos_ball, double k)
{
    const double x_target = -0.66;
    double y_target;
    if (pos_ball.x < -0.55)
        y_target = pos_ball.y;
    else
        y_target = math::bound(pos_ball.y, -0.3, 0.3);
    
    ctrl::vec2 target = {std::min(x_target, pos_ball.x), y_target};

    double phi = apf::vertical_line_field(pos_robot, target, k);
    ctrl::vec2 apf_vec;
    sincos(phi, &apf_vec.y, &apf_vec.x);

    return apf_vec;
}

ctrl::vec2 gpk::kick(bool cw)
{
    const double speed = 150;
    ctrl::vec2 apf_vec;

    if (cw)
    {
        apf_vec = {speed, -speed};
    }
    else
    {
        apf_vec = {-speed, speed};
    }

    return apf_vec;
}