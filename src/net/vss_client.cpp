#include "net/vss_client.h"

VSSClient::VSSClient(int port, std::string ip, bool team_yellow)
{
    this->my_port = port;
    this->my_addr = ip;
    this->team_yellow = team_yellow;
    this->socket = new UDPSocket();
}

VSSClient::~VSSClient()
{
    delete this->socket;
}

/**
 * @brief Send robot i wheel command
 * 
 * @param id 
 * @param wheel_left 
 * @param wheel_right 
 */
void VSSClient::sendCommand(int id, double wheel_left, double wheel_right)
{
    fira_message::sim_to_ref::Packet packet;
    fira_message::sim_to_ref::Command *command = packet.mutable_cmd()->add_robot_commands();

    command->set_yellowteam(this->team_yellow);
    command->set_id(id);
    command->set_wheel_left(wheel_left);
    command->set_wheel_right(wheel_right);

    Datagram dgram(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    this->socket->sendData(dgram, this->my_addr, this->my_port);
}

/**
 * @brief Send multiple commands in one packet. cmd[i] will send command to robot i
 * 
 * @param cmd Vector of control vec2 ([0] is motor left, [1] is motor right)
 */
void VSSClient::sendCommand(std::vector<ctrl::vec2> &cmd)
{
    fira_message::sim_to_ref::Packet packet;

    for (size_t i = 0; i < cmd.size(); i++)
    {
        fira_message::sim_to_ref::Command *command = packet.mutable_cmd()->add_robot_commands();
        command->set_yellowteam(this->team_yellow);
        command->set_id(i);
        command->set_wheel_left(cmd[i][0]);
        command->set_wheel_right(cmd[i][1]);
    }

    Datagram dgram(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    this->socket->sendData(dgram, this->my_addr, this->my_port);
}