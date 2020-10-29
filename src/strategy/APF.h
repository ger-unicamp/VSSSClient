#ifndef APF
#define APF
#include "strategy/controller.h"

namespace apf
{
    // ctrl::vec2 uniform_goal_field();
    // ctrl::vec2 uniform_walls_field(fira_message::Robot &robot);
    ctrl::vec2 repulsion_field(fira_message::Robot &robot, fira_message::Robot &obstacle, double k);
 
    double spiral_field_cw(ctrl::vec2 pos, double radius, double k);
    double spiral_field_ccw(ctrl::vec2 pos, double radius, double k);
    double move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k);
    ctrl::vec2 ball_field(ctrl::vec2 robot, ctrl::vec2 ball, double radius, double k);
}; // namespace apf

#endif