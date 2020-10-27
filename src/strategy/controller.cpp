#include "strategy/controller.h"
#include <iostream>

/**
 * @brief Set motor speed (left, right) to move into vector (x,y) direction
 * 
 * @param robot 
 * @param vector Indicates speed and orientation to follow
 * @param k 
 * @param v 
 * @return ctrl::vec2 
 */
ctrl::vec2 ctrl::move_robot(fira_message::Robot &robot, ctrl::vec2 vector, double k, double v)
{
    // k is the turning gain constant and v is the velocity constant
    double vel = v * vector.abs();
    double angle_diff = math::wrap_to_pi(vector.theta() - robot.orientation());

    ctrl::vec2 motors_speed = vel * cos(angle_diff) * ctrl::vec2(1.0);
    if (angle_diff >= HALF_PI || angle_diff <= -HALF_PI)
        motors_speed += vel * k * sin(angle_diff) * ctrl::vec2(1.0, -1.0);
    else
        motors_speed += vel * k * sin(angle_diff) * ctrl::vec2(-1.0, 1.0);

    return motors_speed;
}
