#include "Midfielder.h"
#include "strategy/Game.h"

Midfielder::Midfielder(fira_message::Robot &robot): Player(robot) {}

/**
 * @brief Midfielder behaviour - returns goalkeeper final motors speeds
 * 
 * @param robots 
 * @return ctrl::vec2 
 */
ctrl::vec2 Midfielder::behaviour(std::vector<fira_message::Robot> &robots){
    fira_message::Robot closest_robot;
    ctrl::vec2 target, univec, motors_speeds;
    double repulsion_phi, sigmoid_phi, phi;

    target = ctrl::vec2(Game::ball);

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