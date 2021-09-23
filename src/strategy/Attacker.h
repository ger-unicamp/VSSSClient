#ifndef ATTACKER_HEADER
#define ATTACKER_HEADER


#include "Player.h"

class Attacker: public Player
{
private:
    static constexpr double RADIUS = 0.0755485;
    static constexpr double K_SPIRAL = 0.0691405;
    
    static constexpr bool CW = true;
    static constexpr bool CCW = false;

    static unsigned int stuck_count;ctrl::vec2 vec;

    double univec_spiral_field(ctrl::vec2 pos, bool is_cw);
    double univec_spiral_field_to_target(ctrl::vec2 target);
    double univec_horizontal_sigmoid_field(ctrl::vec2 target);
    ctrl::vec2 univec_rotate();

public:
    Attacker(fira_message::Robot &robot);

    ctrl::vec2 play(std::vector<fira_message::Robot> &robots);
};

#endif