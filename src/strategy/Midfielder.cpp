#include "Midfielder.h"

Midfielder::Midfielder(fira_message::Robot &robot): Player(robot) {}

unsigned int Midfielder::lock_count = 0;

ctrl::vec2 Midfielder::play(std::vector<fira_message::Robot> &robots)
{
    fira_message::Ball ball = Game::ball;
    if (is_locked(this->lock_count))
    {
        bool cw = this->robot.y() < ball.y();
        return spin(cw);
    }

    fira_message::Robot closest_robot;
    ctrl::vec2 target, univec, motors_speeds;
    double repulsion_phi, sigmoid_phi, phi;

    target = ctrl::vec2(ball);

    target.y = math::bound(target.y, -Midfielder::MID_LIMIT, Midfielder::MID_LIMIT);
    if (target.x >= 0)
    { 
        target.x = math::bound(target.x - Midfielder::MID_LIMIT, -Midfielder::MID_LIMIT, Midfielder::MID_LIMIT);
    }
    else 
    {
        target.x = math::bound(target.x + Midfielder::MID_LIMIT, -Midfielder::MID_LIMIT, Midfielder::MID_LIMIT);
    }

    closest_robot = this->get_closest_robot(robots);
    repulsion_phi = this->univec_repulsion_field(closest_robot);
    sigmoid_phi = this->univec_vertical_sigmoid_field(target);
    phi = this->univec_composite_field(repulsion_phi, sigmoid_phi, this->get_pos().distance(closest_robot));
    sincos(phi, &univec.y, &univec.x);

    motors_speeds = this->move(univec);

    return motors_speeds;
}