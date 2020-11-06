#include <iostream>
#include "referee_client.h"

RefereeClient::RefereeClient(uint32_t recv_port, uint32_t send_port, std::string addr)
{
    this->send_port = send_port;
    this->recv_port = recv_port;
    this->udp_addr = addr;
    this->send_sock = new UDPSocket();
    this->recv_sock = new UDPSocket();
}

RefereeClient::~RefereeClient()
{
    this->close();
}

void RefereeClient::close()
{
    delete this->send_sock;
    delete this->recv_sock;
}

bool RefereeClient::open(bool blocking)
{
    this->recv_sock->listen("0.0.0.0", this->recv_port, blocking, true, true);
    this->recv_sock->joinMulticastGroup(this->udp_addr);
    return true;
}

/**
 * @brief Non-blocking call to receive a package from referee
 * 
 * @param cmd 
 * @return true Receive a package and updates cmd
 * @return false There is no package to receive; cmd might not be updated
 */
bool RefereeClient::receive(VSSRef::ref_to_team::VSSRef_Command &cmd)
{
    Datagram dgram;
    std::string recv_ip;
    unsigned int recv_port;
    if (this->recv_sock->receiveData(dgram, recv_ip, &recv_port) > 0)
    {
        return cmd.ParseFromArray(dgram.data(), dgram.size());
    }
    return false;
}

bool RefereeClient::send(VSSRef::team_to_ref::VSSRef_Placement &cmd)
{
    Datagram dgram(cmd.ByteSize());
    cmd.SerializeToArray(dgram.data(), dgram.size());
    if (this->send_sock->sendData(dgram, this->udp_addr, this->send_port) > 0)
        return true;

    return false;
}