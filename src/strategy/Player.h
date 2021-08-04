#ifndef PLAYER_HEADER
#define PLAYER_HEADER

#include "pb/common.pb.h"
#include "util/vec2.h"
 
class Player
{
protected:
    static constexpr double INF = std::numeric_limits<double>::max();
    static constexpr double K_TURNING = 0.443467;
    static constexpr double K_VEL = 39.9596;
    static constexpr double DT = 0.147302; 
    static constexpr double SPIN_SPEED = 150;
    static constexpr double K_LINE = 16.0; 
    static constexpr double SIGMA = 0.0413777;
    static constexpr double D_MIN = 0.02;
    static constexpr double MIN_MOVING_VEL = 0.1;
    static constexpr int N_FRAMES_IS_STOPPED = 60;
    static constexpr int N_FRAMES_STOP_SPIN = 70;

    // protected constructor
    Player(fira_message::Robot &robot);

    fira_message::Robot robot;
    fira_message::Robot get_closest_robot(std::vector<fira_message::Robot> &robots);
    ctrl::vec2 future_position_relative_to(fira_message::Robot &r, double dt);
    ctrl::vec2 future_position_relative_to(fira_message::Ball &b, double dt);
    ctrl::vec2 future_position();
    double univec_repulsion_field(fira_message::Robot &obstacle);
    double univec_vertical_sigmoid_field(ctrl::vec2 target);
    double univec_composite_field(double repulsion_phi, double spiral_phi, double closest_obstacle_dist);

public:
    fira_message::Robot get_robot();
    void set_robot(fira_message::Robot &robot);
    ctrl::vec2 get_pos();
    double future_dist_to(fira_message::Ball &ball);
    double future_dist_to(fira_message::Robot &robot);
    ctrl::vec2 move( ctrl::vec2 vector);
    ctrl::vec2 spin(bool cw);
    bool is_locked(unsigned int &lock_count);
};

#endif