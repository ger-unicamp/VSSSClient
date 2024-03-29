#ifndef GOALKEEPER_HEADER
#define GOALKEEPER_HEADER

#include "Player.h"

class Goalkeeper: public Player
{
private:
    
    static constexpr double X_LIMIT_GKP = -0.6675;
    static constexpr double Y_LIMIT_GKP = 0.350;
    static constexpr double X_LIMIT_BALL = -0.52;
    static constexpr double DT_GKP = 0.245;
    static constexpr double KICK_DIST = 0.075;

    static unsigned int lock_count;

    ctrl::vec2 defend_goal_from(ctrl::vec2 ball_pos);
    double get_bisector_slope(ctrl::vec2 ball_pos);
    double get_Y_position(ctrl::vec2 ball_pos, double slope);

public:
    Goalkeeper(fira_message::Robot &robot);
    ctrl::vec2 play(fira_message::Ball &ball);
};

#endif