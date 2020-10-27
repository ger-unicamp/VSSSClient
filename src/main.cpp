/**
 * @file main.cpp
 * @author Renato Sousa and Rodrigo Caus
 * @brief 
 * @version 0.1
 * @date 2020-07-09
 * 
 */
#include <stdio.h>
#include "net/robocup_ssl_client.h"
#include "net/vss_client.h"
#include "util/timer.h"
#include "strategy/controller.h"
#include "strategy/APF.h"

#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

void printRobotInfo(const fira_message::Robot &robot)
{

    printf("ID=%3d \n", robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n", robot.x(), robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n", robot.vx(), robot.vy());

    printf("ANGLE=%6.3f \n", robot.orientation());
    printf("ANGLE VEL=%6.3f \n", robot.vorientation());
}

int main(int argc, char *argv[])
{

    RoboCupSSLClient client(10020, "224.0.0.1");
    VSSClient sim_client(20011, "127.0.0.1");

    client.open(false);

    fira_message::sim_to_ref::Environment packet;

    while (true)
    {
        if (client.receive(packet))
        {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_frame())
            {
                fira_message::Frame detection = packet.frame();

                int robots_blue_n = detection.robots_blue_size();
                int robots_yellow_n = detection.robots_yellow_size();

                //Ball info:

                fira_message::Ball ball = detection.ball();
                printf("-Ball:  POS=<%9.2f,%9.2f> \n", ball.x(), ball.y());

                //Blue robot info:
                for (int i = 0; i < robots_blue_n; i++)
                {
                    fira_message::Robot robot = detection.robots_blue(i);
                    printf("-Robot(B) (%2d/%2d): ", i + 1, robots_blue_n);
                    printRobotInfo(robot);

                    // attackers
                    if (i == 0)
                    {
                        ctrl::vec2 apf_vec = apf::ball_field(robot, ball, 0.0737, 0.0415);
                        // ctrl::vec2 apf_vec = apf::test_control(robot, ball);
                        // ctrl::vec2 apf_vec = apf::uniform_goal_field();
                        // apf_vec += apf::uniform_walls_field(robot);
                        
                        // for (int j = 0; j < robots_yellow_n; j++)
                        //     {
                        //         fira_message::Robot enemy = detection.robots_yellow(j);
                        //         apf_vec += apf::robots_field(robot, enemy);
                        //     }

                        // for (int j = 0; j < robots_blue_n; j++)
                        //     {
                        //         if (j != i)
                        //         {    
                        //             fira_message::Robot bro = detection.robots_blue(j);
                        //             apf_vec += apf::robots_field(robot, bro);
                        //         }
                        //     }

                        std::cout << apf_vec.x << apf_vec.y << std::endl;

                        
                        ctrl::vec2 command = apf::move_robot(robot, apf_vec, 0.4, 5);
                        sim_client.sendCommand(i, 10*command[0], 10*command[1]);
                    }

                    // goalkeeper
                    // else if (i == 2)
                    // {
                    //     ctrl::vec2 apf_vec = apf::goalkeeper(robot, ball, 0.0737, 0.0415);

                    //     ctrl::vec2 command = apf::move_robot(robot, apf_vec, 0.4, 5);
                    //     sim_client.sendCommand(i, 10*command[0], 10*command[1]);
                    // }
                    
                }

                //Yellow robot info:
                for (int i = 0; i < robots_yellow_n; i++)
                {
                    fira_message::Robot robot = detection.robots_yellow(i);
                    printf("-Robot(Y) (%2d/%2d): ", i + 1, robots_yellow_n);
                    printRobotInfo(robot);
                }
            }

            //see if packet contains geometry data:
            if (packet.has_field())
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
