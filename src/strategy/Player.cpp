#include "Player.h"

Player::Player(fira_message::Robot &robot): robot(robot) {}

/**
 * @brief Returns Player robot speed (left, right) to move into vector (x,y) direction
 * 
 * @param vector Indicates speed and orientation to follow
 * @return ctrl::vec2 
 */
ctrl::vec2 Player::move(ctrl::vec2 vector)
{
    // k is the turning gain constant and v is the velocity constant
    double vel = Player::K_VEL * vector.abs();
    double angle_diff = math::wrap_to_pi(vector.theta() - this->robot.orientation());

    ctrl::vec2 motors_speed = vel * cos(angle_diff) * ctrl::vec2(1.0);
    if (angle_diff >= HALF_PI || angle_diff <= -HALF_PI)
        motors_speed += vel * Player::K_TURNING * sin(angle_diff) * ctrl::vec2(1.0, -1.0);
    else
        motors_speed += vel * Player::K_TURNING * sin(angle_diff) * ctrl::vec2(-1.0, 1.0);

    return motors_speed;
}

/**
 * @brief checks if Player robot is locked and should spin to get free
 * 
 * @param stopped_count 
 * @return true robot is locked
 * @return false robot isn't locked
 */
bool Player::is_locked(unsigned int &stopped_count)
{
    ctrl::vec2 robot_vel(this->robot.vx(), this->robot.vy());

    if (robot_vel.abs() > 0.1 || stopped_count >= 70) stopped_count = 0;
    else ++stopped_count;

    return (stopped_count >= 60);
}

/**
 * @brief spin Player robot to kick ball or get free from locked position
 * 
 * @param cw rotate clockwise or counterclockwise
 * @return ctrl::vec2 
 */
ctrl::vec2 Player::spin(bool cw)
{
    ctrl::vec2 motors_speed;

    motors_speed = cw ? ctrl::vec2(Player::SPIN_SPEED, -Player::SPIN_SPEED)
                   : ctrl::vec2(-Player::SPIN_SPEED, Player::SPIN_SPEED);

    return motors_speed;
}

/**
 * @brief Returns ball future position based on its relative position to Player
 * 
 * @param b 
 * @return ctrl::vec2 
 */
ctrl::vec2 Player::future_position_of(fira_message::Ball &b, double dt)
{
    ctrl::vec2 fut_pos = dt * ctrl::vec2(b.vx(), b.vy());
    double dist = ctrl::vec2(b).distance(ctrl::vec2(this->robot));
    if (dist < fut_pos.abs())
        fut_pos = dist * fut_pos.normalized();
    
    fut_pos = ctrl::vec2(b) + fut_pos;
    fut_pos.y = math::bound(fut_pos.y, -0.65, 0.65);
    fut_pos.x = math::bound(fut_pos.x, -0.75, 0.75);
    
    return fut_pos;
}

/**
 * @brief Returns robot r future position based on its relative position to Player
 * 
 * @param r
 * @return ctrl::vec2 
 */
ctrl::vec2 Player::future_position_of(fira_message::Robot &r, double dt)
{
    ctrl::vec2 fut_pos = dt * ctrl::vec2(r.vx() - this->robot.vx(), r.vy() - this->robot.vy());
    double dist = ctrl::vec2(r).distance(ctrl::vec2(this->robot));
    if (dist < fut_pos.abs())
        fut_pos = dist * fut_pos.normalized();
    
    fut_pos = ctrl::vec2(r) + fut_pos;
    fut_pos.y = math::bound(fut_pos.y, -0.65, 0.65);
    fut_pos.x = math::bound(fut_pos.x, -0.75, 0.75);

    return fut_pos;
}


