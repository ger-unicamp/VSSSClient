#ifndef APF
#define APF
#include "strategy/controller.h"

namespace apf
{
    ctrl::vec2 uniform_goal_field();
    ctrl::vec2 robots_field(ctrl::vec2 robot1, ctrl::vec2 robot2, double k = 0.20);
    ctrl::vec2 uniform_walls_field(ctrl::vec2 robot);
    ctrl::vec2 test_control(ctrl::vec2 robot, ctrl::vec2 ball);

    double spiral_field_cw(ctrl::vec2 pos, double radius, double k);
    double spiral_field_ccw(ctrl::vec2 pos, double radius, double k);
    double move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k);
    ctrl::vec2 ball_field(ctrl::vec2 robot, ctrl::vec2 ball, double radius, double k);
}; // namespace apf

#endif