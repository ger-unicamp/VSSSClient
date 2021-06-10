// #ifndef UNIVECTOR_HEADER
// #define UNIVECTOR_HEADER

// #include <vector>
// #include <utility>

// #include "net/vss_client.h"

// #include "util/vec2.h"s

// namespace univec
// {
//     constexpr double RADIUS = 0.0755485;
//     constexpr double K_SPIRAL = 0.0691405;
//     constexpr double SIGMA = 0.0413777;
//     constexpr double D_MIN = 0.02;
//     constexpr double K_LINE = 16.0;
//     constexpr double GOAL_X = 0.75;
//     constexpr double GOAL_Y = 0.0;

//     // BREAK REPULSION FIELD INTO FUNCTIONS
//     std::pair<double, double> repulsion_field(unsigned int moving_robot_id, std::vector<fira_message::Robot> my_robots, std::vector<fira_message::Robot> enemy_robots);
//     double spiral_field_cw(ctrl::vec2 pos);
//     double spiral_field_ccw(ctrl::vec2 pos);
//     double spiral_field_to_target(ctrl::vec2 pos, ctrl::vec2 target);
//     double composite_field(double repulsion_phi, double spiral_phi);
//     double vertical_line_field(ctrl::vec2 pos, ctrl::vec2 target);
//     double horizontal_line_field(ctrl::vec2 pos, ctrl::vec2 target);
// };

// #endif