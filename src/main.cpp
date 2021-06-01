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

#include "strategy/controller.h"
#include "strategy/APF.h"
#include "strategy/goalkeeper.h"
#include "strategy/roles.h"

namespace {
    unsigned int stopped_count_gkp = 0;
    unsigned int stopped_count_atk = 0;
    unsigned int stopped_count_def = 0;
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

void startup(int argc, char **argv, net_config &conf, bool &team_yellow);

void detect_objects(fira_message::Frame detection, fira_message::Ball &ball,
                    vector<fira_message::Robot> &my_robots, vector<fira_message::Robot> &enemy_robots, bool yellow);

bool answer_referee(VSSRef::ref_to_team::VSSRef_Command &ref_packet, VSSRef::team_to_ref::VSSRef_Placement &cmd, bool yellow);

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

    VSSRef::team_to_ref::VSSRef_Placement cmd;

    while (true)
    {
        if (referee.receive(ref_packet))
        {
            game_on = answer_referee(ref_packet, cmd, yellow);

            stopped_count_gkp = 0;
            stopped_count_atk = 0;
            stopped_count_def = 0;

            referee.send(cmd);
        }

        if (client.receive(packet) && packet.has_frame())
        {
            fira_message::Frame detection = packet.frame();

            detect_objects(detection, ball, my_robots, enemy_robots, yellow);

            std::vector<ctrl::vec2> commands;
            if (!game_on)
            {
                commands = {{0, 0}, {0, 0}, {0, 0}};
            }
            else
            {
                commands = rol::select_role(ball, my_robots, enemy_robots);
            }

            sim_client.sendCommand(commands);
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

void replace_robots(VSSRef::Robot *robot0, VSSRef::Robot *robot1, VSSRef::Robot *robot2, vector<ctrl::vec2> &positions)
{
    robot0->set_x(positions[0].x);
    robot0->set_y(positions[0].y);
    robot0->set_orientation(90);
    robot1->set_x(positions[1].x);
    robot1->set_y(positions[1].y);
    robot1->set_orientation(0);
    robot2->set_x(positions[2].x);
    robot2->set_y(positions[2].y);
    robot2->set_orientation(0);
}

bool answer_referee(VSSRef::ref_to_team::VSSRef_Command &ref_packet, VSSRef::team_to_ref::VSSRef_Placement &cmd, bool yellow)
{
    VSSRef::Frame *replacement = new VSSRef::Frame();

    if (yellow) replacement->set_teamcolor(VSSRef::YELLOW);
    else replacement->set_teamcolor(VSSRef::BLUE);
    
    auto robot0 = replacement->add_robots();
    robot0->set_robot_id(0);
    auto robot1 = replacement->add_robots();
    robot1->set_robot_id(1);
    auto robot2 = replacement->add_robots();
    robot2->set_robot_id(2);    
    unsigned int foul = ref_packet.foul();
    std::cout << "-----Referee Foul: " << foul << std::endl;

    bool game_on = false;

    vector<ctrl::vec2> positions;

    switch (foul)
    {
    case VSSRef::GAME_ON:
        game_on = true;
        break;

    case VSSRef::STOP:
        game_on = false;
        break;

    case VSSRef::FREE_BALL: 
        if (ref_packet.foulquadrant() == VSSRef::QUADRANT_2 || ref_packet.foulquadrant() == VSSRef::QUADRANT_3)
        {
            if (!yellow) positions = {{-0.7, 0.0}, {-0.6, 0.4}, {-0.6, -0.4}};
            else positions = {{0.7, 0.0}, {-0.2, -0.4}, {-0.2, 0.4}};
        }

        else if (ref_packet.foulquadrant() == VSSRef::QUADRANT_1 || ref_packet.foulquadrant() == VSSRef::QUADRANT_4)
        {
            if (!yellow) positions = {{-0.7, 0.0}, {0.175, 0.4}, {0.175, -0.4}};
            else positions = {{0.7, 0.0}, {0.55, -0.4}, {0.55, 0.4}};
        }
        replace_robots(robot0, robot1, robot2, positions);
        break;
    
    case VSSRef::PENALTY_KICK:
        if (ref_packet.teamcolor() == VSSRef::BLUE)
        {
            if (!yellow) positions = {{-0.7, 0.0}, {0.25, 0.04}, {-0.1, 0.0}};
            else positions = {{0.7, 0.0}, {-0.1, -0.25}, {-0.1, 0.25}};
        }

        else if (ref_packet.teamcolor() == VSSRef::YELLOW)
        {
            if (!yellow) positions = {{-0.7, 0.0}, {0.1, 0.25}, {0.1, -0.25}};
            else positions = {{0.7, 0.0}, {-0.25, -0.04}, {0.1, 0.0}};
        }
        replace_robots(robot0, robot1, robot2, positions);
        break;

    case VSSRef::KICKOFF:
        if (!yellow) positions = {{-0.7, 0.0}, {-0.25, 0.0}, {-0.4, -0.2}};
        else positions = {{0.7, 0.0}, {0.25, 0.0}, {0.4, 0.2}};
        replace_robots(robot0, robot1, robot2, positions);
        
        break;

    case VSSRef::GOAL_KICK:
        if (ref_packet.teamcolor() == VSSRef::BLUE)
        {   
            if (!yellow)
            {
                positions = {{-0.65, -0.3}, {-0.4, -0.25}, {-0.5, -0.55}};
                replace_robots(robot0, robot1, robot2, positions);
                robot0->set_orientation(135);
            }
            else
            {
                positions = {{0.7, 0.0}, {-0.25, -0.2}, {-0.25, 0.2}};
                replace_robots(robot0, robot1, robot2, positions);
            }   
        }

        else if (ref_packet.teamcolor() == VSSRef::YELLOW)
        {
            if (!yellow)
            {
                positions = {{-0.7, 0.0}, {0.25, 0.2}, {0.25, -0.2}};
                replace_robots(robot0, robot1, robot2, positions);
            }
            else
            {
                positions = {{0.7, 0.25}, {0.4, 0.25}, {0.5, 0.55}};
                replace_robots(robot0, robot1, robot2, positions);
                robot0->set_orientation(135);
            }
        }
        break;         
    }

    cmd.set_allocated_world(replacement);

    return game_on;
}