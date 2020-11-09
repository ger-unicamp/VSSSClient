#ifndef APF
#define APF
#include "strategy/controller.h"
#include <algorithm>
#include <vector>

namespace apf
{
    ctrl::vec2 uniform_walls_field(ctrl::vec2 robot);
    std::pair<double, double> apf::repulsion_field(unsigned int moving_robot_id, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots);

    double spiral_field_cw(ctrl::vec2 pos, double radius, double k);
    double spiral_field_ccw(ctrl::vec2 pos, double radius, double k);
    double move_to_goal(ctrl::vec2 pos, ctrl::vec2 target, double radius, double k);
    ctrl::vec2 ball_field(ctrl::vec2 robot, ctrl::vec2 ball, double radius, double k);
}; // namespace apf

#endif