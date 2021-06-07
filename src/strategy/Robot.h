#ifndef ROBOT_HEADER
#define ROBOT_HEADER

#include "net/vss_client.h"

class Robot: public fira_message::Robot
{
    static constexpr double K_TURNING = 0.443467;
    static constexpr double K_VEL = 39.9596;
    static constexpr double DT = 0.147302;
    static constexpr double DT_GOALKEEPER = 0.245;

public:
    Robot();

    ctrl::vec2 move_robot(ctrl::vec2 vector);
    ctrl::vec2 future_position(Robot &robot);
    ctrl::vec2 future_position(fira_message::Ball &ball);
};

#endif