#ifndef GOALKEEPER_HEADER
#define GOALKEEPER_HEADER

#include "Player.h"

class Goalkeeper: public Player
{
private:
    static unsigned int lock_count;

public:
    static constexpr double GKP_X_AXIS = -0.7;
    static constexpr double Y_LIMIT_GKP = 0.275;
    static constexpr double X_LIMIT_BALL = -0.52;
    static constexpr double DT_GKP = 0.245;
    static constexpr double KICK_DIST = 0.075;

    Goalkeeper(fira_message::Robot &robot);
    ctrl::vec2 play(fira_message::Ball &ball);
};

#endif