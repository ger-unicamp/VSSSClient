#ifndef UNIVECTOR_HEADER
#define UNIVECTOR_HEADER

#include "util/vec2.h"

class UnivectorField 
{
    static constexpr double RADIUS = 0.0755485;
    static constexpr double K_SPIRAL = 0.0691405;
    static constexpr double K_TURNING = 0.443467;
    static constexpr double K_VEL = 39.9596;
    static constexpr double DT = 0.147302;
    static constexpr double DT_GOALKEEPER = 0.245;
    static constexpr double SIGMA = 0.0413777;
    static constexpr double D_MIN = 0.02;
    static constexpr double K_LINE = 16.0;
    static constexpr double GOAL_X = 0.75;
    static constexpr double GOAL_Y = 0.0;

    UnivectorField();

    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 vector);
    ctrl::vec2 future_position(fira_message::Robot &r1, fira_message::Robot &r2);
    ctrl::vec2 future_position(fira_message::Ball &b, fira_message::Robot &r);
};

#endif