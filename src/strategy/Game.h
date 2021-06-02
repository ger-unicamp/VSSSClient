#ifndef GAME_HEADER
#define GAME_HEADER

#include <iostream>
#include <algorithm>

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"
#include "pb/vssref_command.pb.h"
#include "pb/vssref_common.pb.h"
#include "pb/vssref_placement.pb.h"

#include "net/robocup_ssl_client.h"
#include "net/vss_client.h"
#include "net/referee_client.h"

#include "util/timer.h"
#include "util/argparse.h"


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
    bool is_yellow;

    net_config conf;

    fira_message::Ball ball;
    vector<fira_message::Robot> my_robots;
    vector<fira_message::Robot> enemy_robots;

    Game(bool is_yellow, int argc, char *argv[]);

    void startup(int argc, char **argv);
    
    vector<fira_message::Robot> detect_robots(bool is_yellow, fira_message::Frame frame);
    fira_message::Ball detect_ball(fira_message::Frame frame);
    void detect_objects(fira_message::Frame frame);

    void play(); // TODO
};

#endif