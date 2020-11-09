#ifndef APF
#define APF
#include "strategy/controller.h"
#include <algorithm>
#include <vector>

namespace apf
{
    ctrl::vec2 walls_field(fira_message::Robot &robot);

    std::pair<double, double> repulsion_field(unsigned int moving_robot_id, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots);
    double composite_field(double repulsion_phi, double spiral_phi, double sigma, double dmin, double R);
    
    double spiral_field_cw(ctrl::vec2 pos, double radius, double k);
    double spiral_field_ccw(ctrl::vec2 pos, double radius, double k);
    double move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k);
    ctrl::vec2 ball_field(ctrl::vec2 robot,  ctrl::vec2 ball, double radius, double k);
};


#endif