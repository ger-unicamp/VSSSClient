#include "UnivectorField.h"

/**
 * @brief Set motor speed (left, right) to move into vector (x,y) direction
 * 
 * @param robot 
 * @param vector Indicates speed and orientation to follow
 * @param k 
 * @param v 
 * @return ctrl::vec2 
 */
ctrl::vec2 UnivectorField::move_robot(fira_message::Robot &robot, ctrl::vec2 vector)
{
    // k is the turning gain constant and v is the velocity constant
    double vel = UnivectorField::K_VEL * vector.abs();
    double angle_diff = math::wrap_to_pi(vector.theta() - robot.orientation());

    ctrl::vec2 motors_speed = vel * cos(angle_diff) * ctrl::vec2(1.0);

    // choose between clockwise and clockwise rotation depending on robot-ball angle 
    if (angle_diff >= HALF_PI || angle_diff <= -HALF_PI)
        motors_speed += vel * UnivectorField::K_TURNING * sin(angle_diff) * ctrl::vec2(1.0, -1.0);
    else
        motors_speed += vel * UnivectorField::K_TURNING * sin(angle_diff) * ctrl::vec2(-1.0, 1.0);

    return motors_speed;
}