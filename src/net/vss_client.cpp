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
