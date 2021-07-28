#ifndef SIM_CLIENT
#define SIM_CLIENT

#include <vector>

#include "net/udpsocket.h"
#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

#include "util/vec2.h"

class VSSClient
{
    uint16_t my_port;
    std::string my_addr;
    bool team_yellow;
    UDPSocket *socket;

public:
    VSSClient(int port = 20011, std::string ip = "127.0.0.1", bool team_yellow = false);
    ~VSSClient();

    void sendCommand(int id, double wheel_left, double wheel_right);
    void sendCommand(std::vector<ctrl::vec2> &cmd);
    uint16_t getPort() { return this->my_port; }
    std::string getAddr() { return this->my_addr; }
    bool amIYellow() { return this->team_yellow; }
    void setPort(uint16_t port) { this->my_port = port; }
    void setAddr(std::string addr) { this->my_addr = addr; }

};

#endif