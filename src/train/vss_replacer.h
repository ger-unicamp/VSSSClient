#ifndef SIM_REPLACER
#define SIM_REPLACER

#include "net/udpsocket.h"
#include "pb/command.pb.h"
#include "pb/common.pb.h"
#include "pb/packet.pb.h"
#include "pb/replacement.pb.h"

class VSSReplacer
{
    uint16_t my_port;
    std::string my_addr;
    UDPSocket *socket;

public:
    VSSReplacer(int port = 20011, std::string ip = "127.0.0.1");
    ~VSSReplacer();
    void setRobotPos(bool yellow, int id, double x, double y, double orientation);
    void setBallPos(double x, double y);
    void setRobotsOutside(bool yellow, size_t n);
    uint16_t getPort() { return this->my_port; }
    std::string getAddr() { return this->my_addr; }
    void setPort(uint16_t port) { this->my_port = port; }
    void setAddr(std::string addr) { this->my_addr = addr; }

};

#endif