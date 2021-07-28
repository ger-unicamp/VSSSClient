#ifndef ATTACKER_HEADER
#define ATTACKER_HEADER

#include "Player.h"

class Attacker: public Player
{
private:
    static constexpr double RADIUS = 0.0755485;
    static constexpr double K_SPIRAL = 0.0691405;
    static constexpr double FRIENDLY_GOAL_X_LIMIT = 0.5;
    static constexpr bool CW = true;
    static constexpr bool CCW = false;

    static unsigned int lock_count;

    double univec_spiral_field(ctrl::vec2 pos, bool is_cw);
    double univec_spiral_field_to_target(ctrl::vec2 target);
    double univec_horizontal_sigmoid_field(ctrl::vec2 target);

public:
    Attacker(fira_message::Robot &robot);

    ctrl::vec2 play(fira_message::Ball &ball, std::vector<fira_message::Robot> &robots);
};

#endif