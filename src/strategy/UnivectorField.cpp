#include "UnivectorField.h"

/**
 * @brief returns robot's motor speed (left, right) for moving into vector (x,y) direction
 * 
 * @param robot 
 * @param vector indicates speed and orientation to follow
 * @return ctrl::vec2 
 */
ctrl::vec2 UnivectorField::move_robot(fira_message::Robot &robot, ctrl::vec2 vector)
{
    // k is the turning gain constant and v is the velocity constant
    double vel = UnivectorField::K_VEL * vector.abs();
    double angle_diff = math::wrap_to_pi(vector.theta() - robot.orientation());

    ctrl::vec2 motors_speed = vel * cos(angle_diff) * ctrl::vec2(1.0);

    // choose between clockwise or anticlockwise rotation depending on angle between robot and ball
    if (angle_diff >= HALF_PI || angle_diff <= -HALF_PI)
        motors_speed += vel * UnivectorField::K_TURNING * sin(angle_diff) * ctrl::vec2(1.0, -1.0);
    else
        motors_speed += vel * UnivectorField::K_TURNING * sin(angle_diff) * ctrl::vec2(-1.0, 1.0);

    return motors_speed;
}

/**
 * @brief returns ball's future position based on its relative position to rotrl
 * 
 * @param b ball
 * @param r robot
 * @return ctrl::vec2 
 */
ctrl::vec2 UnivectorField::future_position(fira_message::Ball &b, fira_message::Robot &r)
{
    ctrl::vec2 path = UnivectorField::DT * ctrl::vec2(b.vx(), b.vy());
    double dist = ctrl::vec2(b).distance(ctrl::vec2(r));
    if (dist < path.abs())
        path = dist * path.normalized();
    
    path = ctrl::vec2(b) + path;
    path.y = math::bound(path.y, -0.65, 0.65);
    path.x = math::bound(path.x, -0.75, 0.75);
    
    return path;
}

/**
 * @brief returns robot 1's future position based on its relative position to robot 2
 * 
 * @param r1 robot 1
 * @param r2 robot 2
 * @return ctrl::vec2 
 */
ctrl::vec2 UnivectorField::future_position(fira_message::Robot &r1, fira_message::Robot &r2)
{
    ctrl::vec2 path = UnivectorField::DT * ctrl::vec2(r1.vx() - r2.vx(), r1.vy() - r2.vy());
    double dist = ctrl::vec2(r1).distance(ctrl::vec2(r2));
    if (dist < path.abs())
        path = dist * path.normalized();
    
    path = ctrl::vec2(r1) + path;
    path.y = math::bound(path.y, -0.65, 0.65);
    path.x = math::bound(path.x, -0.75, 0.75);

    return path;
}