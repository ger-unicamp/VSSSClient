#include "Midfielder.h"

Midfielder::Midfielder(fira_message::Robot &robot): Player(robot) {}

unsigned int Midfielder::lock_count = 0;

ctrl::vec2 Midfielder::play(fira_message::Ball &ball, std::vector<fira_message::Robot> robots)
{
    if (is_locked(this->lock_count))
    {
        bool cw = this->robot.y() < ball.y();
        return spin(cw);
    }

    fira_message::Robot closest_robot;
    ctrl::vec2 target, univec, motors_speeds;
    double repulsion_phi, sigmoid_phi, phi;

    target = ctrl::vec2(ball);

    target.y = math::bound(target.y, -0.4, 0.4);
    if (target.x >= 0)
    { 
        target.x = math::bound(target.x - 0.4, -0.4, 0.4);
    }
    else 
    {
        target.x = math::bound(target.x + 0.4, -0.4, 0.4);
    }

    closest_robot = this->get_closest_robot(robots);
    repulsion_phi = this->univec_repulsion_field(closest_robot);
    sigmoid_phi = this->univec_vertical_sigmoid_field(target);
    phi = this->univec_composite_field(repulsion_phi, sigmoid_phi, this->get_pos().distance(closest_robot));
    sincos(phi, &univec.y, &univec.x);

    motors_speeds = this->move(univec);

    return motors_speeds;
}