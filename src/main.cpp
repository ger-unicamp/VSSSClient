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
#include <vector>
#include <map>
#include "net/robocup_ssl_client.h"
#include "net/vss_client.h"
#include "net/referee_client.h"
#include "util/timer.h"
#include "strategy/controller.h"
#include "strategy/APF.h"
#include "strategy/goalkeeper.h"

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"
#include "pb/vssref_command.pb.h"
#include "pb/vssref_common.pb.h"
#include "pb/vssref_placement.pb.h"

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

pair<double, double> compose_repulsion(unsigned int moving_robot_id, vector<fira_message::Robot> my_robots, vector<fira_message::Robot> enemy_robots)
{
    int n_obstacles = 0;
    double repulsion_phi;
    double repulsion_phi_sum = 0;
    double dist;
    std::vector<double> distances;

    for (size_t i = 0; i < my_robots.size(); ++i)
    {
        if (i != moving_robot_id)
        {
            auto obstacle =  ctrl::future_position(my_robots[i], my_robots[moving_robot_id], 0.147302);
            ++n_obstacles;
            repulsion_phi = (ctrl::vec2(my_robots[moving_robot_id]) - obstacle).theta();
            repulsion_phi_sum += repulsion_phi;
            dist = ctrl::vec2(my_robots[moving_robot_id]).distance(obstacle);
            distances.push_back(dist);
        }
    }
    for (size_t i = 0; i < enemy_robots.size(); ++i)
    {
        auto obstacle =  ctrl::future_position(enemy_robots[i], my_robots[moving_robot_id], 0.147302);
        ++n_obstacles;
        repulsion_phi = (ctrl::vec2(my_robots[moving_robot_id]) - obstacle).theta();
        repulsion_phi_sum += repulsion_phi;
        dist = ctrl::vec2(my_robots[moving_robot_id]).distance(obstacle);
        distances.push_back(dist);
    }

    auto min_dist = std::min_element(distances.begin(), distances.end());
    return make_pair(repulsion_phi_sum/n_obstacles, *min_dist);
}

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

                // G0:0.0755485 G1:0.0691405 G2:0.443467 G3: 40 * 0.664899 + 10
                auto robot = my_robots[0];;
                double spiral_phi = apf::move_to_goal(robot, ball, 0.0755485, 0.0691405);
                pair<double, double> tmp = compose_repulsion(0, my_robots, enemy_robots);
                double phi = apf::composite_field(tmp.first, spiral_phi, 0.0413777, 0.0290801, tmp.second);
                ctrl::vec2 apf_vec;
                sincos(phi, &apf_vec.y, &apf_vec.x);
                ctrl::vec2 command = ctrl::move_robot(robot, apf_vec, 0.443467, 39.9596);
                sim_client.sendCommand(0, command[0], command[1]);
                // if (game_on)
                //     sim_client.sendCommand(0, command[0], command[1]);
                // else
                //     sim_client.sendCommand(0, 0.0, 0.0);
 
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