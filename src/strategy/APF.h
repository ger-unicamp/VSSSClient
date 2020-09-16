#ifndef APF
#define APF
#include "strategy/controller.h"


namespace apf
{
    ctrl::vec2 move_robot(fira_message::Robot &robot, ctrl::vec2 apf_vector);
    ctrl::vec2 uniform_goal_field();
    ctrl::vec2 robots_field(fira_message::Robot &robot1, fira_message::Robot &robot2);
    ctrl::vec2 ball_field(fira_message::Robot &robot, fira_message::Ball &ball);
    ctrl::vec2 uniform_walls_field(fira_message::Robot &robot); 
};


#endif