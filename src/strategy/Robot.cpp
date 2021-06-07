#include "Robot.h"

/**
 * @brief returns robot's motor speed (left, right) for moving into vector (x,y) direction
 * 
 * @param robot 
 * @param vector indicates speed and orientation to follow
 * @return ctrl::vec2 
 */
ctrl::vec2 Robot::move_robot(ctrl::vec2 vector)
{
    // k is the turning gain constant and v is the velocity constant
    double vel = Robot::K_VEL * vector.abs();
    double angle_diff = math::wrap_to_pi(vector.theta() - this->orientation());

    ctrl::vec2 motors_speed = vel * cos(angle_diff) * ctrl::vec2(1.0);

    // choose between clockwise or anticlockwise rotation depending on angle between robot and ball
    if (angle_diff >= HALF_PI || angle_diff <= -HALF_PI)
        motors_speed += vel * Robot::K_TURNING * sin(angle_diff) * ctrl::vec2(1.0, -1.0);
    else
        motors_speed += vel * Robot::K_TURNING * sin(angle_diff) * ctrl::vec2(-1.0, 1.0);

    return motors_speed;
}

/**
 * @brief returns ball's future position based on its relative position to this robot
 * 
 * @param ball ball
 * @return ctrl::vec2 
 */
ctrl::vec2 Robot::future_position(fira_message::Ball &ball)
{
    ctrl::vec2 fut_pos = Robot::DT * ctrl::vec2(ball.vx(), ball.vy());
    double dist = ctrl::vec2(ball).distance(ctrl::vec2(this));
    if (dist < fut_pos.abs())
        fut_pos = dist * fut_pos.normalized();
    
    fut_pos = ctrl::vec2(ball) + fut_pos;
    fut_pos.y = math::bound(fut_pos.y, -0.65, 0.65);
    fut_pos.x = math::bound(fut_pos.x, -0.75, 0.75);
    
    return fut_pos;
}

/**
 * @brief returns robot's future position based on its relative position to this robot
 * 
 * @param robot
 * @return ctrl::vec2 
 */
ctrl::vec2 Robot::future_position(Robot &robot)
{
    ctrl::vec2 fut_pos = Robot::DT * ctrl::vec2(this->vx() - robot.vx(), this->vy() - robot.vy());
    double dist = ctrl::vec2(this).distance(ctrl::vec2(robot));
    if (dist < fut_pos.abs())
        fut_pos = dist * fut_pos.normalized();
    
    fut_pos = ctrl::vec2(this) + fut_pos;
    fut_pos.y = math::bound(fut_pos.y, -0.65, 0.65);
    fut_pos.x = math::bound(fut_pos.x, -0.75, 0.75);

    return fut_pos;
}