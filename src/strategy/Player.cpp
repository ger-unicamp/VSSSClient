#include "Player.h"

Player::Player(fira_message::Robot &robot): robot(robot) {}

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

/**
 * @brief univector field that converge to target with final orientation in vertical
 * 
 * @param target 
 * @return double 
 */
double Player::univec_vertical_sigmoid_field(ctrl::vec2 target)
{
    double phi;

    // univector always operate over translated points
    ctrl::vec2 translated = this->get_pos() - target;

    if (translated.y <= 0.0)
        phi = PI * (1.0 / (1.0 + std::exp(-1.0 * Player::K_LINE * translated.x)));
    else
        phi = PI * (-1.0 / (1.0 + std::exp(-1.0 * Player::K_LINE * translated.x)));

    return phi;
}

/**
 * @brief get closest robot r from Player
 * 
 * @param my_robots 
 * @param enemy_robots 
 * @return fira_message::Robot 
 */
fira_message::Robot Player::get_closest_robot(std::vector<fira_message::Robot> robots)
{
    double min_dist = Player::INF;
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
 * @param obstacle 
 * @return double 
 */
double Player::univec_repulsion_field(fira_message::Robot obstacle)
{
    ctrl::vec2 fut_obstacle = this->future_position_of(obstacle, Player::DT);
    double phi = (this->get_pos() - fut_obstacle).theta();
    return phi;
}

/**
 * @brief composes repulsion and move to goal fields with gaussian compound ratio
 * 
 * @param repulsion_phi 
 * @param spiral_phi
 * @return double final composed phi
 */
double Player::univec_composite_field(double repulsion_phi, double spiral_phi, double closest_obstacle_dist)
{
    double gauss = math::gaussian(closest_obstacle_dist - Player::D_MIN, Player::SIGMA);
    if (closest_obstacle_dist <= Player::D_MIN)
        return repulsion_phi;
    else
        return (repulsion_phi * gauss + spiral_phi * (1-gauss));
}

// --------------------> PUBLIC FUNCS <--------------------

ctrl::vec2 Player::get_pos()
{
    return ctrl::vec2(this->robot);
}

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

    ctrl::vec2 motors_speeds = vel * cos(angle_diff) * ctrl::vec2(1.0);
    if (angle_diff >= HALF_PI || angle_diff <= -HALF_PI)
        motors_speeds += vel * Player::K_TURNING * sin(angle_diff) * ctrl::vec2(1.0, -1.0);
    else
        motors_speeds += vel * Player::K_TURNING * sin(angle_diff) * ctrl::vec2(-1.0, 1.0);

    return motors_speeds;
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
    ctrl::vec2 motors_speeds;

    motors_speeds = cw ? ctrl::vec2(Player::SPIN_SPEED, -Player::SPIN_SPEED)
                      : ctrl::vec2(-Player::SPIN_SPEED, Player::SPIN_SPEED);

    return motors_speeds;
}

