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

/**
 * @brief Returns b's future position based on its relative position to r
 * 
 * @param b 
 * @param r 
 * @param dt Time interval in seconds
 * @return ctrl::vec2 
 */
ctrl::vec2 ctrl::future_position(fira_message::Ball &b, fira_message::Robot &r, double dt)
{
    ctrl::vec2 path = dt * ctrl::vec2(b.vx(), b.vy());
    double dist = ctrl::vec2(b).distance(ctrl::vec2(r));
    if (dist < path.abs())
        path = dist * path.normalized();
    
    path = ctrl::vec2(b) + path;
    path.y = math::bound(path.y, -0.65, 0.65);
    path.x = math::bound(path.x, -0.75, 0.75);
    
    return path;
}

/**
 * @brief Returns r1's future position based on its relative position to r2
 * 
 * @param r1 
 * @param r2 
 * @param dt Time interval in seconds
 * @return ctrl::vec2 
 */
ctrl::vec2 ctrl::future_position(fira_message::Robot &r1, fira_message::Robot &r2, double dt)
{
    ctrl::vec2 path = dt * ctrl::vec2(r1.vx() - r2.vx(), r1.vy() - r2.vy());
    double dist = ctrl::vec2(r1).distance(ctrl::vec2(r2));
    if (dist < path.abs())
        path = dist * path.normalized();
    
    path = ctrl::vec2(r1) + path;
    path.y = math::bound(path.y, -0.65, 0.65);
    path.x = math::bound(path.x, -0.75, 0.75);

    return path;
}
