#ifndef CONTROLLER_HEADER
#define CONTROLEER_HEADER

#include "net/vss_client.h"

#include "util/vec2.h"

namespace ctrl 
{
    constexpr double K_TURNING = 0.443467;
    constexpr double K_VEL = 39.9596;
    constexpr double DT = 0.147302; 
    constexpr double DT_GOALKEEPER = 0.245;   
    
    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 vector);
    ctrl::vec2 future_position(fira_message::Robot &r1, fira_message::Robot &r2);
    ctrl::vec2 future_position(fira_message::Ball &b, fira_message::Robot &r);
};

#endif