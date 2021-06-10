#ifndef ATTACKER_HEADER
#define ATTACKER_HEADER

#include "Player.h"

class Attacker: public Player
{
private:
    static constexpr double RADIUS = 0.0755485;
    static constexpr double K_SPIRAL = 0.0691405;
    static constexpr double SIGMA = 0.0413777;
    static constexpr double D_MIN = 0.02;
    static constexpr double K_LINE = 16.0;
    static constexpr double GOAL_X = 0.75;
    static constexpr double GOAL_Y = 0.0;

    fira_message::Robot get_closest_robot(std::vector<fira_message::Robot> robots);
    double univec_repulsion_field(fira_message::Robot obstacle);
    double univec_spiral_field(ctrl::vec2 pos, char cw);
    double univec_spiral_field_cw(ctrl::vec2 pos);
    double univec_spiral_field_ccw(ctrl::vec2 pos);
    double univec_spiral_field_to_target(ctrl::vec2 target);
    double univec_horizontal_line_field(ctrl::vec2 target);
    double univec_composite_field(double repulsion_phi, double spiral_phi);

public:
    Attacker(fira_message::Robot &robot);

    ctrl::vec2 play(fira_message::Ball &ball); // TODO for main attacker and secondary
};

#endif