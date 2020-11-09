/**
 * @file main.cpp
 * @author Renato Sousa and Rodrigo Caus
 * @brief 
 * @version 0.1
 * @date 2020-07-09
 * 
 */
#include <stdio.h>
#include <algorithm>
#include "util/timer.h"
#include "util/argparse.h"
#include "net/robocup_ssl_client.h"
#include "net/vss_client.h"
#include "net/referee_client.h"
#include "strategy/controller.h"
#include "strategy/APF.h"
#include "strategy/goalkeeper.h"
#include "strategy/roles.h"

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"
#include "pb/vssref_command.pb.h"
#include "pb/vssref_common.pb.h"
#include "pb/vssref_placement.pb.h"



void detect_objects(fira_message::Frame detection, fira_message::Ball &ball,
                    vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots, bool yellow);

int main(int argc, char *argv[])
{
    bool yellow = false;

    RoboCupSSLClient client(10002, "224.0.0.1");
    VSSClient sim_client(20011, "127.0.0.1", yellow);
    RefereeClient referee(10003, 10004, "224.0.0.1");

    client.open(false);
    referee.open();

    fira_message::sim_to_ref::Environment packet;
    VSSRef::ref_to_team::VSSRef_Command ref_packet;
    bool game_on = true;

    fira_message::Ball ball;
    vector<fira_message::Robot> my_robots;
    vector<fira_message::Robot> enemy_robots;

    while (true)
    {
        if (referee.receive(ref_packet))
        {
            unsigned int foul = ref_packet.foul();
            std::cout << "-----Referee Foul: " << foul << std::endl;
            game_on = foul == 6;
        }

        if (client.receive(packet))
        {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_frame())
            {
                fira_message::Frame detection = packet.frame();

                detect_objects(detection, ball, my_robots, enemy_robots, yellow);

                std::vector<ctrl::vec2> commands;
                if (!game_on)
                {
                    commands = {{0,0},{0,0},{0,0}};
                }
                else
                {
                    commands = rol::select_role(ball,my_robots,enemy_robots);
                }
                
                for (size_t i = 0; i < commands.size(); i++)
                {
                    sim_client.sendCommand(i,commands[i][0],commands[i][1]);
                }
                
                
            }
            
        }
    }

    return 0;
}

void printRobotInfo(const fira_message::Robot &robot)
{

    printf("ID=%3d \n", robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n", robot.x(), robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n", robot.vx(), robot.vy());

    printf("ANGLE=%6.3f \n", robot.orientation());
    printf("ANGLE VEL=%6.3f \n", robot.vorientation());
}

fira_message::Ball invert_ball(fira_message::Ball ball)
{
    ball.set_x(-ball.x());
    ball.set_y(-ball.y());
    ball.set_vx(-ball.vx());
    ball.set_vy(-ball.vy());

    return ball;
}

fira_message::Robot invert_robot(fira_message::Robot robot)
{
    robot.set_x(-robot.x());
    robot.set_y(-robot.y());
    robot.set_vx(-robot.vx());
    robot.set_vy(-robot.vy());
    robot.set_orientation(math::wrap_to_pi(robot.orientation() + PI));
    robot.set_vorientation(-robot.vorientation());

    return robot;
}

void print_info(fira_message::Ball ball, vector<fira_message::Robot> my_robots,
                vector<fira_message::Robot> enemy_robots, bool yellow = false)
{
    //Ball info:
    printf("-Ball:  POS=<%9.2f,%9.2f> \n", ball.x(), ball.y());
    printf("-Ball:  VEL=<%9.2f,%9.2f> \n", ball.vx(), ball.vy());

    // playing with blue team
    if (!yellow)
    {
        //Blue robot info:
        for (auto robot : my_robots)
        {
            printf("-Robot(B) (%2d/%2lu): ", robot.robot_id() + 1, my_robots.size());
            printRobotInfo(robot);
        }

        //Yellow robot info:
        for (auto robot : enemy_robots)
        {
            printf("-Robot(Y) (%2d/%2lu): ", robot.robot_id() + 1, enemy_robots.size());
            printRobotInfo(robot);
        }
    }

    // playing with yellow team
    else
    {
        //Blue robot info:
        for (auto robot : enemy_robots)
        {
            printf("-Robot(B) (%2d/%2lu): ", robot.robot_id() + 1, enemy_robots.size());
            printRobotInfo(robot);
        }

        //Yellow robot info:
        for (auto robot : my_robots)
        {
            printf("-Robot(Y) (%2d/%2lu): ", robot.robot_id() + 1, my_robots.size());
            printRobotInfo(robot);
        }
    }
}

void detect_objects(fira_message::Frame detection, fira_message::Ball &ball,
                    vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots, bool yellow = false)
{
    int robots_blue_n = detection.robots_blue_size();
    int robots_yellow_n = detection.robots_yellow_size();

    ball = detection.ball();

    // playing with blue team
    if (!yellow)
    {
        my_robots.resize(robots_blue_n);
        enemy_robots.resize(robots_yellow_n);

        for (int i = 0; i < robots_blue_n; ++i)
        {
            my_robots[i] = detection.robots_blue(i);
        }

        for (int i = 0; i < robots_yellow_n; ++i)
        {
            enemy_robots[i] = detection.robots_yellow(i);
        }
    }

    // playing with yellow team
    else
    {
        enemy_robots.resize(robots_blue_n);
        my_robots.resize(robots_yellow_n);

        for (int i = 0; i < robots_blue_n; ++i)
        {
            enemy_robots[i] = detection.robots_blue(i);
        }

        for (int i = 0; i < robots_yellow_n; ++i)
        {
            my_robots[i] = detection.robots_yellow(i);
        }

        // invert ball and robots
        ball = invert_ball(ball);
        std::for_each(my_robots.begin(), my_robots.end(),
                      [](fira_message::Robot &robot) {
                          robot = invert_robot(robot);
                      });

        std::for_each(enemy_robots.begin(), enemy_robots.end(),
                      [](fira_message::Robot &robot) {
                          robot = invert_robot(robot);
                      });
    }

    //print_info(ball, my_robots, enemy_robots, yellow);
}

struct net_config
{
    std::string multicast_ip;
    std::string command_ip;
    unsigned int command_port;
    unsigned int referee_port;
    unsigned int replacer_port;
    unsigned int vision_port;
};

void startup(int argc, char **argv, net_config &conf, bool &team_yellow)
{
    ArgParse::ArgumentParser parser(argv[0], "GER VSSS FIRASim strategy server");
    parser.add_argument('m', "multicast_ip", "Vision and Referee IP", new ArgParse::value<std::string>("224.0.0.1"));
    parser.add_argument('c', "command_ip", "Command IP", new ArgParse::value<std::string>("127.0.0.1"));
    parser.add_argument('d', "command_port", "Command port", new ArgParse::value<unsigned int>("20011"));
    parser.add_argument('e', "referee_port", "Referee foul port", new ArgParse::value<unsigned int>("10003"));
    parser.add_argument('r', "replacer_port", "Referee command port", new ArgParse::value<unsigned int>("10004"));
    parser.add_argument('v', "vision_port", "Vision port", new ArgParse::value<unsigned int>("10002"));
    parser.add_argument('t', "team_yellow", "Team collor yellow (true/false)", new ArgParse::value<bool>("false"));
    parser.add_argument('h', "help", "Show help menu");

    auto args = parser.parse_args(argc, argv);

    if(args["help"]->as<bool>())
    {
        std::cout << parser.help() << std::endl;
        exit(0);
    }

    conf.multicast_ip = args["multicast_ip"]->as<std::string>();
    conf.command_ip = args["command_ip"]->as<std::string>();
    conf.command_port = args["command_port"]->as<unsigned int>();
    conf.referee_port = args["referee_port"]->as<unsigned int>();
    conf.replacer_port = args["replacer_port"]->as<unsigned int>();
    conf.vision_port = args["vision_port"]->as<unsigned int>();
    team_yellow = args["team_yellow"]->as<bool>();

    // Print startup configuration
    std::cout << "Vision server at " << conf.multicast_ip << ":" << conf.vision_port << std::endl
              << "Command listen to " << conf.command_ip << ":" << conf.command_port << std::endl
              << "Referee server at " << conf.multicast_ip << ":" << conf.referee_port << std::endl
              << "Replacer listen to " << conf.multicast_ip << ":" << conf.replacer_port << std::endl
              << "Color team: " << (team_yellow? "yellow":"blue") << std::endl;

}

int main(int argc, char *argv[])
{
    bool yellow = false;
    net_config conf;
    startup(argc, argv, conf, yellow);

    RoboCupSSLClient client(conf.vision_port, conf.multicast_ip);
    VSSClient sim_client(conf.command_port, conf.command_ip, yellow);
    RefereeClient referee(conf.referee_port, conf.replacer_port, conf.multicast_ip);

    client.open(false);
    referee.open();

    fira_message::sim_to_ref::Environment packet;
    VSSRef::ref_to_team::VSSRef_Command ref_packet;
    bool game_on = true;

    fira_message::Ball ball;
    vector<fira_message::Robot> my_robots;
    vector<fira_message::Robot> enemy_robots;

    while (true)
    {
        if (referee.receive(ref_packet))
        {
            unsigned int foul = ref_packet.foul();
            std::cout << "-----Referee Foul: " << foul << std::endl;
            game_on = foul == 6;
        }

        if (client.receive(packet))
        {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_frame())
            {
                fira_message::Frame detection = packet.frame();

                detect_objects(detection, ball, my_robots, enemy_robots, yellow);

                // G0: 0.125666 G1:0.0695225 G2:0.392803 G3:0.822646
                ctrl::vec2 apf_vec = apf::ball_field(my_robots[0], ball, 0.125666, 0.0695225);
                ctrl::vec2 command = ctrl::move_robot(my_robots[0], apf_vec, 0.392803, 40.0 * 0.822646 + 10.0);
                if (game_on)
                    sim_client.sendCommand(0, command[0], command[1]);
                else
                    sim_client.sendCommand(0, 0.0, 0.0);

                if (!game_on)
                    sim_client.sendCommand(2, 0.0, 0.0);
                else if (ctrl::vec2(my_robots[2]).distance(ball) < 0.08)
                {
                    ctrl::vec2 spin = gpk::kick(my_robots[2], ball);
                    sim_client.sendCommand(2, spin[0], spin[1]);
                }
                else
                {
                    apf_vec = gpk::follow(my_robots[2], ball);
                    command = ctrl::move_robot(my_robots[2], apf_vec, 0.4, 5);
                    sim_client.sendCommand(2, 10 * command[0], 10 * command[1]);
                }
            }

            //see if packet contains geometry data:
            if (packet.has_field() && false)
            {
                printf("-[Geometry Data]-------\n");

                const fira_message::Field &field = packet.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%f (mm)\n", field.length());
                printf("  -field_width=%f (mm)\n", field.width());
                printf("  -goal_width=%f (mm)\n", field.goal_width());
                printf("  -goal_depth=%f (mm)\n", field.goal_depth());
            }
        }
    }

    return 0;
}
