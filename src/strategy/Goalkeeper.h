#ifndef GOALKEEPER_HEADER
#define GOALKEEPER_HEADER

#include "Player.h"

class Goalkeeper: Player
{
private:
    static constexpr double X_LIMIT_GKP = -0.66;
    static constexpr double Y_LIMIT_GKP = 0.28;
    static constexpr double X_LIMIT_BALL = -0.55;
    static constexpr double K_LINE = 16.0; 
    static constexpr double DT_GKP = 0.245;
    static constexpr double KICK_DIST = 0.077;

    double univec_vertical_line_field(ctrl::vec2 target);
    ctrl::vec2 defend_goal_from(ctrl::vec2 ball_pos);

public:
    Goalkeeper(fira_message::Robot &robot);
    ctrl::vec2 play(fira_message::Ball &ball);
};

#endif