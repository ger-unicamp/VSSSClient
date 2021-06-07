#ifndef UNIVECTOR_HEADER
#define UNIVECTOR_HEADER

#include "util/vec2.h"

class UnivectorField 
{
    static constexpr double RADIUS = 0.0755485;
    static constexpr double K_SPIRAL = 0.0691405;
    static constexpr double SIGMA = 0.0413777;
    static constexpr double D_MIN = 0.02;
    static constexpr double K_LINE = 16.0;
    static constexpr double GOAL_X = 0.75;
    static constexpr double GOAL_Y = 0.0;

    UnivectorField();
};

#endif