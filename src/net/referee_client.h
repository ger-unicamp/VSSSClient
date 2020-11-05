#ifndef REF_CLIENT_H
#define REF_CLIENT_H

#include <string>
#include "net/udpsocket.h"
#include "pb/vssref_command.pb.h"
#include "pb/vssref_common.pb.h"
#include "pb/vssref_placement.pb.h"

class RefereeClient
{
    uint32_t recv_port, send_port;
    std::string udp_addr;
    UDPSocket *recv_sock;
    UDPSocket *send_sock;

public:
    RefereeClient(uint32_t recv_port, uint32_t send_port, std::string addr);
    ~RefereeClient();
    bool open(bool blocking = false);
    void close();
    bool receive(VSSRef::ref_to_team::VSSRef_Command &cmd);
    bool send(VSSRef::team_to_ref::VSSRef_Placement &cmd);
};

#endif