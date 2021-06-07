#ifndef UNIVECTOR_HEADER
#define UNIVECTOR_HEADER

#include <vector>
#include <utility>

#include "net/vss_client.h"

#include "util/vec2.h"

class UnivectorField 
{
    static constexpr double K_TURNING = 0.443467;
    static constexpr double K_VEL = 39.9596;
    static constexpr double DT = 0.147302;
    
    static constexpr double DT_GOALKEEPER = 0.245;
    static constexpr double RADIUS = 0.0755485;
    static constexpr double K_SPIRAL = 0.0691405;
    static constexpr double SIGMA = 0.0413777;
    static constexpr double D_MIN = 0.02;
    static constexpr double K_LINE = 16.0;
    static constexpr double GOAL_X = 0.75;
    static constexpr double GOAL_Y = 0.0;

    UnivectorField();

    std::pair<double, double> repulsion_field(unsigned int moving_robot_id, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots);
    double spiral_field_cw(ctrl::vec2 pos);
    double spiral_field_ccw(ctrl::vec2 pos);
    double spiral_field_to_target(ctrl::vec2 pos, ctrl::vec2 target);
    double composite_field(double repulsion_phi, double spiral_phi);
    double vertical_line_field(ctrl::vec2 pos, ctrl::vec2 target);
    double horizontal_line_field(ctrl::vec2 pos, ctrl::vec2 target);
};

#endif