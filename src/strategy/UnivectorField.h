#ifndef UNIVECTOR_HEADER
#define UNIVECTOR_HEADER

#include "util/vec2.h"

class UnivectorField 
{
    static const double RADIUS = 0.0755485;
    static const double K_SPIRAL = 0.0691405;
    static const double K_TURNING = 0.443467;
    static const double K_VEL = 39.9596;
    static const double DT = 0.147302;
    static const double DT_GOALKEEPER = 0.245;
    static const double SIGMA = 0.0413777;
    static const double D_MIN = 0.02;
    static const double K_LINE = 16.0;

    UnivectorField();

    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 vector);
    ctrl::vec2 future_position(fira_message::Robot &r1, fira_message::Robot &r2); // TODO
    ctrl::vec2 future_position(fira_message::Ball &b, fira_message::Robot &r); // TODO
};

#endif