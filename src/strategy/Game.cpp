#include "strategy/Game.h"
#include "util/argparse.h"

Game::Game(bool is_yellow, int argc, char *argv[]) 
{
    this->startup(argc, argv);
    this->is_yellow = is_yellow;
}

/**
 * @brief Set network config and team color
 * 
 * @param argc 
 * @param argv 
 */
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

fira_message::Ball Game::detect_ball(fira_message::Frame frame) 
{
    return frame.ball();
}

/**
 * @brief returns blue or yellow vector of robots
 * 
 * @param is_yellow 
 * @param frame 
 * @return vector<fira_message::Robot> 
 */
vector<fira_message::Robot> Game::detect_robots(bool is_yellow, fira_message::Frame frame) 
{
    vector<fira_message::Robot> robots;
    uint robots_size = is_yellow ? frame.robots_yellow_size() : frame.robots_blue_size();
    robots.resize(robots_size);

    for (uint i = 0; i < robots_size; ++i) 
    {
        robots[i] = is_yellow ? frame.robots_yellow(i) : frame.robots_blue(i);
    }

    return robots;
}

/**
 * @brief stores state of current game objects into Game attributes
 * 
 * @param frame current game frame
 */
void Game::detect_objects(fira_message::Frame frame) 
{
    this->ball = detect_ball(frame);
    this->my_robots = detect_robots(this->is_yellow, frame);
    this->enemy_robots = detect_robots(!this->is_yellow, frame);
}

void Game::play()
{
    RoboCupSSLClient client(this->conf.vision_port, this->conf.multicast_ip);
    VSSClient sim_client(this->conf.command_port, this->conf.command_ip, this->is_yellow);
    RefereeClient referee(this->conf.referee_port, this->conf.replacer_port, this->conf.multicast_ip);

    client.open(false); // opens client
    referee.open(); // opens referee

    fira_message::sim_to_ref::Environment packet;
    VSSRef::ref_to_team::VSSRef_Command ref_packet;
    VSSRef::team_to_ref::VSSRef_Placement cmd;

    while (true) 
    {
        if (referee.receive(ref_packet))
        {

            referee.send(cmd);
        }

        if (client.receive(packet) && packet.has_frame())
        {
            fira_message::Frame detection = packet.frame();

            detect_objects(detection);

            std::vector<ctrl::vec2> commands;

            commands = {{0, 0}, {0, 0}, {0, 0}};

            sim_client.sendCommand(commands);
        }
    }
}