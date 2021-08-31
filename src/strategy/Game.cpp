#include <iostream>
#include "strategy/Game.h"
#include "util/argparse.h"
#include "net/referee_client.h"

Game::Game(int argc, char *argv[]) 
{
    this->startup(argc, argv);
}

/* **********************************************
 * @brief Set network config and team color
 * 
 * @param argc 
 * @param argv 
 * *********************************************/
void Game::startup(int argc, char **argv) 
{
    ArgParse::ArgumentParser parser(argv[0], "GER VSSS FIRASim strategy server");
    parser.add_argument('m', "multicast_ip", "Vision and Referee IP", ArgParse::value<std::string>("224.0.0.1"));
    parser.add_argument('c', "command_ip", "Command IP", ArgParse::value<std::string>("127.0.0.1"));
    parser.add_argument('d', "command_port", "Command port", ArgParse::value<unsigned int>(20011));
    parser.add_argument('e', "referee_port", "Referee foul port", ArgParse::value<unsigned int>(10003));
    parser.add_argument('r', "replacer_port", "Referee command port", ArgParse::value<unsigned int>(10004));
    parser.add_argument('v', "vision_port", "Vision port", ArgParse::value<unsigned int>(10002));
    parser.add_argument('t', "team_yellow", "Team collor yellow (true/false)", ArgParse::value<bool>(false));
    parser.add_argument('h', "help", "Show help menu");

    auto args = parser.parse_args(argc, argv);

    if (args["help"]->as<bool>())
    {
        std::cout << parser.help() << std::endl;
        exit(0);
    }

    this->conf.multicast_ip = args["multicast_ip"]->as<std::string>();
    this->conf.command_ip = args["command_ip"]->as<std::string>();
    this->conf.command_port = args["command_port"]->as<unsigned int>();
    this->conf.referee_port = args["referee_port"]->as<unsigned int>();
    this->conf.replacer_port = args["replacer_port"]->as<unsigned int>();
    this->conf.vision_port = args["vision_port"]->as<unsigned int>();
    this->is_yellow = args["team_yellow"]->as<bool>();

    // Print startup configuration
    std::cout << "Vision server at " << this->conf.multicast_ip << ":" << this->conf.vision_port << std::endl
              << "Command listen to " << this->conf.command_ip << ":" << this->conf.command_port << std::endl
              << "Referee server at " << this->conf.multicast_ip << ":" << this->conf.referee_port << std::endl
              << "Replacer listen to " << this->conf.multicast_ip << ":" << this->conf.replacer_port << std::endl
              << "Color team: " << (this->is_yellow ? "yellow":"blue") << std::endl;
}

/* *********************************************************
 * @brief takes the ball's positions x and y, 
 *        as well as its vx and vy velocities
 *        and multiply both of them by (-1) 
 * 
 * @obs   There's no need to invert after assignement.
 *        This doensn't need to be so complex
 * *********************************************************/
void Game::invert_ball()
{
    ball.set_x(-ball.x());      // invert positon x
    ball.set_y(-ball.y());      // invert positon y
    ball.set_vx(-ball.vx());    // invert speed   vx
    ball.set_vy(-ball.vy());    // invert speed   vy
}

/* *********************************************************
 * @brief takes the robot's positions x and y, 
 *        as well as its vx and vy velocities and its 
 *        orientation on the field and multiply both of
 *        them by (-1) 
 * 
 * @obs   There's no need to invert after assignement.
 *        This doensn't need to be so complex
 * *********************************************************/
fira_message::Robot Game::invert_robot(fira_message::Robot &robot)
{
    robot.set_x(-robot.x());     // invert positon x
    robot.set_y(-robot.y());     // invert positon y
    robot.set_vx(-robot.vx());   // invert speed   vx
    robot.set_vy(-robot.vy());   // invert speed   vy
    robot.set_orientation(math::wrap_to_pi(robot.orientation() + PI));  // invert orientation
    robot.set_vorientation(-robot.vorientation());                      // invert speed orientation
    
    return robot;
}

//There's no need to invert after assignement. This doensn't need to be so complex
void Game::invert_field_if_yellow()
{
    if (is_yellow)
    {
        invert_ball();

        // DOES THIS REALLY NEED TO USE LAMBDA
        std::for_each(my_robots.begin(), my_robots.end(),
                     [this](fira_message::Robot &robot) {
                           robot = invert_robot(robot);
                      });
        std::for_each(enemy_robots.begin(), enemy_robots.end(),
                     [this](fira_message::Robot &robot) {
                           robot = invert_robot(robot);
                      });
    }
}

/* ****************************************************************
 * @brief returns vector of blue or yellow robots
 * 
 * @param is_yellow 
 * @param frame 
 * @return vector<Robot>
 * ***************************************************************/
vector<fira_message::Robot> Game::detect_robots(bool is_yellow, fira_message::Frame frame) 
{
    vector<fira_message::Robot> robots;

// Ternary command: If the identified robot is yellow, it will mark the robots_size as team yellow.
// Else, it will mark the robots_size as team blue.
    uint robots_size = is_yellow ? frame.robots_yellow_size() : frame.robots_blue_size(); 
    robots.resize(robots_size);

// Runs a for command to assign each robots_size prevviously marked to its respective robot
    for (uint i = 0; i < robots_size; ++i) 
    {
        robots[i] = is_yellow ? frame.robots_yellow(i) : frame.robots_blue(i);
    }

    return robots;
}

/* *****************************************************************************
 * @brief stores state of current game objects into Game attributes
 * 
 * @param frame current game frame
 * *****************************************************************************/
void Game::detect_objects(fira_message::Frame &frame) 
{
    this->ball = frame.ball();                                        // detect ball
    this->my_robots = detect_robots(this->is_yellow, frame);          // detect my robot
    this->enemy_robots = detect_robots(!this->is_yellow, frame);      // detect enemy robot

    // TODO MOVE THIS
    invert_field_if_yellow();
    this->robots = my_robots;
    this->robots.insert(robots.end(), enemy_robots.begin(), enemy_robots.end());
}

/* ***********************************************************************
 * @brief return closest robots index to ball
 * 
 * @return int
 * ************************************************************************/
unsigned int Game::robot_next_to_ball(Attacker &atk, Midfielder &mid)
{
    double atk_ball_dist = atk.future_dist_to(ball);  // distance between ball and attacker
    double mid_ball_dist = mid.future_dist_to(ball);  // distance between ball and midfielder

    return (mid_ball_dist < atk_ball_dist) ?          // ternary question:
            mid.get_robot().robot_id() :              // if midfielder is closest,  return midfielder's ID
            atk.get_robot().robot_id();               // else, attacker is closest, return attacker's   ID
}

/* **************************************************************************
 * @brief switch atk and mid depending on which one is closer to ball
 * @todo dynamic gkp
 * **************************************************************************/
void Game::select_roles(Goalkeeper &gkp, Attacker &atk, Midfielder &mid) 
{
    int new_atk_id = robot_next_to_ball(atk, mid);
    gkp.set_robot(my_robots[0]); // gkp is always robot 0
    atk.set_robot(my_robots[new_atk_id]);
    mid.set_robot(my_robots[3 - new_atk_id]);
}

// TODO POLYMORPHISM WITH PLAYER
void Game::send_commands(VSSClient &sim_client) 
{
    Goalkeeper gkp(my_robots[0]); // goalkeeper is robot number 0
    Attacker atk(my_robots[1]);   // attacker   is robot number 1
    Midfielder mid(my_robots[2]); // midfielder is robot number 2

    // FUNCTION HAS MORE THAN ONE FUNCTIONALITY
    select_roles(gkp, atk, mid);
    
    ctrl::vec2 gkp_command = gkp.play(this->ball);
    ctrl::vec2 atk_command = atk.play(this->ball, this->robots);
    ctrl::vec2 mid_command = mid.play(this->ball, this->robots);

    sim_client.sendCommand(gkp.get_robot().robot_id(), gkp_command.x, gkp_command.y); // goalkeeper receives its command from strategy 
    sim_client.sendCommand(atk.get_robot().robot_id(), atk_command.x, atk_command.y); // attacker   receives its command from strategy 
    sim_client.sendCommand(mid.get_robot().robot_id(), mid_command.x, mid_command.y); // midfielder receives its command from strategy 
}

void Game::run()
{
    RoboCupSSLClient client(this->conf.vision_port, this->conf.multicast_ip);
    VSSClient sim_client(this->conf.command_port, this->conf.command_ip, this->is_yellow);

    // TODO: REFEREE
    RefereeClient referee(this->conf.referee_port, this->conf.replacer_port, this->conf.multicast_ip);

    client.open(false); // opens client

    fira_message::sim_to_ref::Environment packet;

    while (true)  // this while detect objets in real time and then send commands 
    {
        if (client.receive(packet) && packet.has_frame())
        {
            fira_message::Frame detection = packet.frame();

            detect_objects(detection);
        
            send_commands(sim_client);
        }
    }
}
