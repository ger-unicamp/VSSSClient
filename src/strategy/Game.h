#ifndef GAME_HEADER
#define GAME_HEADER

#include <iostream>
#include <algorithm>

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

#include "net/robocup_ssl_client.h"
#include "net/referee_client.h"

#include "util/timer.h"

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

    fira_message::Ball ball;
    vector<fira_message::Robot> my_robots;
    vector<fira_message::Robot> enemy_robots;
    vector<fira_message::Robot> robots;

    void startup(int argc, char **argv);
    vector<fira_message::Robot> detect_robots(bool is_yellow, fira_message::Frame frame);
    fira_message::Ball detect_ball(fira_message::Frame frame);
    void detect_objects(fira_message::Frame frame);
    unsigned int robot_next_to_ball(Attacker &atk, Midfielder &mid);
    void select_roles(Goalkeeper &gkp, Attacker &atk, Midfielder &mid);
    void send_commands(VSSClient &sim_client);

public:
    Game(int argc, char *argv[]);
    void run();
};

#endif