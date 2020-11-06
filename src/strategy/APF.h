#ifndef APF
#define APF
#include "strategy/controller.h"

namespace apf
{
    // ctrl::vec2 uniform_goal_field();
    // ctrl::vec2 uniform_walls_field(fira_message::Robot &robot);

    // double composite_field(ctrl::vec2 robot, ctrl::vec2 obstacle, double phi_target, double phi_repulsion, double d_min, double sigma);
    double composite_field(ctrl::vec2 repulsion_vec, ctrl::vec2 spiral_vec, double sigma);

    // double repulsion_field(fira_message::Robot &robot, fira_message::Robot &obstacle, double k);
    ctrl::vec2 repulsion_field(ctrl::vec2 robot, ctrl::vec2 obstacle, double k);
    
    double spiral_field_cw(ctrl::vec2 pos, double radius, double k);
    double spiral_field_ccw(ctrl::vec2 pos, double radius, double k);
    double move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k);
    ctrl::vec2 ball_field(fira_message::Robot &robot, fira_message::Ball &ball, double radius, double k);

    ctrl::vec2 composite_field(fira_message::Robot &robot, fira_message::Ball &target, fira_message::Robot &obstacle, double radius, double k_spiral, double k_virtual_obj, double d_min, double sigma);
};


#endif