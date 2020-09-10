#ifndef APF
#define APF
#include "strategy/controller.h"


namespace apf
{
    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 apf_vector);
    ctrl::vec2 goal_field(fira_message::Robot &robot);
    ctrl::vec2 enemies_field(fira_message::Robot &robot, fira_message::Robot &enemy);
    ctrl::vec2 bros_field(fira_message::Robot &robot, fira_message::Robot &bro);
    ctrl::vec2 ball_field(fira_message::Robot &robot, fira_message::Ball &ball);
    ctrl::vec2 walls_field(fira_message::Robot &robot); 
};


#endif