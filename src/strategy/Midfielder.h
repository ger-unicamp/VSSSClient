#ifndef MIDFIELDER_HEADER
#define MIDFIELDER_HEADER

#include "Player.h"

class Midfielder: public Player
{
private:
    static constexpr double MID_LIMIT = 0.4;
    
public:
    Midfielder(fira_message::Robot &robot);
    ctrl::vec2 behaviour(std::vector<fira_message::Robot> &robots);
};

#endif