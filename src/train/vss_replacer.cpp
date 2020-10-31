#include "train/vss_replacer.h"

VSSReplacer::VSSReplacer(int port, std::string ip)
{
    this->my_port = port;
    this->my_addr = ip;
    this->socket = new UDPSocket();
}

VSSReplacer::~VSSReplacer()
{
    delete this->socket;
}

fira_message::Robot *replace_robot(int id, double x, double y, double orientation)
{
    auto robot = new fira_message::Robot();
    robot->set_robot_id(id);
    robot->set_x(x);
    robot->set_y(y);
    robot->set_orientation(orientation);
    robot->set_vx(0.0);
    robot->set_vy(0.0);
    robot->set_vorientation(0.0);
    return robot;
}

void VSSReplacer::setRobotPos(bool yellow, int id, double x, double y, double orientation)
{
    fira_message::sim_to_ref::Packet packet;
    fira_message::sim_to_ref::RobotReplacement *replace = packet.mutable_replace()->add_robots();

    auto robot = replace_robot(id, x, y, orientation);
    replace->set_yellowteam(yellow);
    replace->set_turnon(true);
    replace->set_allocated_position(robot);

    Datagram dgram(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    this->socket->sendData(dgram, this->my_addr, this->my_port);
}

void VSSReplacer::setBallPos(double x, double y)
{
    fira_message::sim_to_ref::Packet packet;
    auto replace = new fira_message::sim_to_ref::BallReplacement();
    replace->set_x(x);
    replace->set_y(y);
    replace->set_vx(0.0);
    replace->set_vy(0.0);

    packet.mutable_replace()->set_allocated_ball(replace);
    Datagram dgram(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    this->socket->sendData(dgram, this->my_addr, this->my_port);
}

void VSSReplacer::setRobotsOutside(bool yellow, size_t n)
{
    fira_message::sim_to_ref::Packet packet;

    for (size_t i = 0; i < n; i++)
    {
        auto replace = packet.mutable_replace()->add_robots();
        double x = 0.5 - (2 * i) / 10.0;
        if (!yellow)
            x = -x;
        auto robot = replace_robot(i, x, -1.0, 0.0);
        replace->set_yellowteam(yellow);
        replace->set_turnon(true);
        replace->set_allocated_position(robot);
    }

    Datagram dgram(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    this->socket->sendData(dgram, this->my_addr, this->my_port);
}
