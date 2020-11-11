#ifndef ROLES
#define ROLES
#include "strategy/controller.h"

using namespace std;

const double RADIUS = 0.0755485;
const double K_SPIRAL = 0.0691405;
const double K_TURNING = 0.443467;
const double K_VEL = 39.9596;
const double DT = 0.147302;
const double SIGMA = 0.0413777;
const double D_MIN = 0.0457;
const double K_LINE = 16.0;

namespace rol
{
    unsigned int robot_next_to_target(unsigned int id_gkp, vector<fira_message::Robot> my_robots, ctrl::vec2 target);
    ctrl::vec2 goalkeeper(fira_message::Robot &robot, fira_message::Ball &ball);
    ctrl::vec2 attacker(unsigned int moving_robot_id, fira_message::Ball &ball, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots);
    ctrl::vec2 defender(unsigned int moving_robot_id, ctrl::vec2 pos_target, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots);
    vector<ctrl::vec2> select_role(fira_message::Ball &ball,vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots);
};


#endif