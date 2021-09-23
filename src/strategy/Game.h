#ifndef GAME_HEADER
#define GAME_HEADER

#include "net/robocup_ssl_client.h"
#include "net/vss_client.h"

#include "strategy/Goalkeeper.h"
#include "strategy/Attacker.h"
#include "strategy/Midfielder.h"

struct net_config
{
    std::string multicast_ip;
    std::string command_ip;
    unsigned int command_port;
    unsigned int referee_port;
    unsigned int replacer_port;
    unsigned int vision_port;
};

class Game 
{ 
private:
    bool is_yellow;

    net_config conf;

    
    vector<fira_message::Robot> my_robots;
    vector<fira_message::Robot> enemy_robots;
    vector<fira_message::Robot> robots;

    void startup(int argc, char **argv);
    void invert_ball();
    fira_message::Robot invert_robot(fira_message::Robot &robot);
    void invert_field_if_yellow();
    vector<fira_message::Robot> detect_robots(bool is_yellow, fira_message::Frame frame);
    void detect_objects(fira_message::Frame &frame);
    unsigned int robot_next_to_ball(Attacker &atk, Midfielder &mid);
    void select_roles(Goalkeeper &gkp, Attacker &atk, Midfielder &mid);
    void send_commands(VSSClient &sim_client);

public:
    static constexpr double FRAMERATE = 1/60;
    static constexpr double ATTACKER_LINE_X = 0.5;
    static constexpr double GOAL_LINE_X = 0.75;
    static constexpr double FIELD_LIMIT_X = 0.90;
    static constexpr double FIELD_LIMIT_Y = 0.65;
    static fira_message::Ball ball;
    static ctrl::vec2 get_ball_future_position(double DT);
    
    Game(int argc, char *argv[]);
    void run();
};

#endif