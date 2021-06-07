#ifndef CONTROLLER_HEADER
#define CONTROLEER_HEADER

#include "net/vss_client.h"

#include "util/vec2.h"

class Controller
{
    static constexpr double K_TURNING = 0.443467;
    static constexpr double K_VEL = 39.9596;
    static constexpr double DT = 0.147302;
    
    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 vector);
    ctrl::vec2 future_position(fira_message::Robot &r1, fira_message::Robot &r2);
    ctrl::vec2 future_position(fira_message::Ball &b, fira_message::Robot &r);
};

#endif