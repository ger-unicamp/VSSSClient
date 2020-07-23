#ifndef UDP_SOCKET
#define UDP_SOCKET

#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <stdexcept>

#include <netdb.h>
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

class Datagram {
    /**
     * @brief   Basic Datagram class to manage data
     *          transferred via UDP.
     * 
     */
public:
    Datagram();
    Datagram(size_t n);
    Datagram(uint8_t *bytes, size_t n);
    ~Datagram();

    uint8_t *data() {return this->bytes;}
    size_t size() {return this->byte_size;}
    uint8_t *resize(size_t n);
    void setdata(uint8_t *bytes, size_t n);
    void fill(uint8_t val);

private:
    uint8_t *bytes;
    size_t byte_size;
};

class UDPSocketRuntimeError: public std::runtime_error {
public:
    UDPSocketRuntimeError(const char *e): std::runtime_error(e) {}
};

class UDPSocket {
    /**
     * @brief   UDP basic socket. Default function is a UDP client
     *          where a remote server is registered.
     * 
     */
public:
    UDPSocket();
    ~UDPSocket();

    void serve(std::string &my_ip, unsigned int my_port = 0);
    int sendData(Datagram &d, const std::string &remote_ip, const unsigned int port);
    int sendDataHostname(Datagram &d, const std::string &remote_name, const unsigned int port);
    int receiveData(Datagram &d, std::string &remote_ip, unsigned int *port);
    bool hasPendingData();

private:
    void _pre_bind(struct sockaddr_in *addr, const std::string &ip, unsigned int port);
    std::string _resolve_hostname(const std::string &hostname);

    int sockfd;
    const static unsigned int BUF_LEN = 65536;
    uint8_t recv_buffer[BUF_LEN];
    int recv_len;
    struct sockaddr_in remote_addr;
    struct sockaddr_in my_addr;
};

#endif