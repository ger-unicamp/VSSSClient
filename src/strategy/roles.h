#ifndef ROLES
#define ROLES
#include "strategy/controller.h"


namespace rol
{
    ctrl::vec2 goalkeeper(fira_message::Robot &robot, fira_message::Ball &ball);
    ctrl::vec2 attacker(fira_message::Robot &robot, fira_message::Ball &ball);
    ctrl::vec2 defender(fira_message::Robot &robot, fira_message::Ball &ball,int i);
};


#endif