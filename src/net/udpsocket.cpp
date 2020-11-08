#include <fcntl.h>
#include "net/udpsocket.h"

Datagram::Datagram()
{
    Datagram(1);
}

Datagram::Datagram(size_t n) : bytes(n, 0)
{
}

Datagram::Datagram(uint8_t *bytes, size_t n)
{
    this->bytes.insert(this->bytes.begin(), bytes, bytes + n);
}

Datagram::~Datagram()
{
    this->bytes.clear();
}

uint8_t *Datagram::resize(size_t n)
{
    this->bytes.resize(n, 0);
    return this->data();
}

/**
 * @brief Set datagram data
 * 
 * @param bytes 
 * @param n 
 */
void Datagram::setdata(uint8_t *bytes, size_t n)
{
    this->bytes.clear();
    this->bytes.insert(this->bytes.begin(), bytes, bytes + n);
}

/**
 * @brief Fill Datagram with var
 * 
 * @param var 
 */
void Datagram::fill(uint8_t var)
{
    this->bytes = std::vector<uint8_t>(this->bytes.size(), var);
}

UDPSocket::UDPSocket()
{
    this->sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (this->sockfd == -1)
    {
        throw UDPSocketRuntimeError("Problem creating UDP socket.");
    }
    this->recv_len = 0;
}

UDPSocket::~UDPSocket()
{
    close(this->sockfd);
}

void UDPSocket::_pre_bind(struct sockaddr_in *addr, const std::string &ip, unsigned int port)
{
    std::memset(addr, 0, sizeof(*addr));
    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    if (inet_aton(ip.c_str(), &(addr->sin_addr)) == 0)
    {
        std::string e = "IP address not valid: " + ip;
        throw UDPSocketRuntimeError(e.c_str());
    }
}

std::string UDPSocket::_resolve_hostname(const std::string &host)
{
    struct hostent *hp = gethostbyname(host.c_str());
    if (hp == 0 || hp->h_addr_list[0] == 0)
    {
        std::string e = "Could not obtain address of " + host;
        throw UDPSocketRuntimeError(e.c_str());
    }
    char ip[16];
    unsigned char *a = (unsigned char *)hp->h_addr_list[0];
    sprintf(ip, "%d.%d.%d.%d", a[0], a[1], a[2], a[3]);

    return std::string(ip);
}

/**
 * @brief Bind a socket to listen on my_ip:my_port
 * 
 * @param my_ip 
 * @param my_port 
 * @param blocking Set IO as blocking or not
 * @param multicast_share Set socket to reuse addr
 * @param multicast_loopback Set socket to receive packets on this host
 */
void UDPSocket::listen(std::string my_ip, unsigned int my_port, bool blocking, bool multicast_share, bool multicast_loopback)
{

    this->_pre_bind(&(this->my_addr), my_ip, my_port);

    int flags = fcntl(this->sockfd, F_GETFL, 0);
    if (flags < 0)
        flags = 0;
    fcntl(this->sockfd, F_SETFL, flags | (blocking ? 0 : O_NONBLOCK));

    int reuse = 1;
    if (multicast_share && setsockopt(this->sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)))
        std::cerr << "Error to set SO_REUSEADDR on UDP socket" << std::endl;

    int yes = 1;
    if (multicast_loopback && setsockopt(this->sockfd, IPPROTO_IP, IP_MULTICAST_LOOP, &yes, sizeof(yes)))
        std::cerr << "Error to set IP_MULTICAST_LOOP on UDP socket" << std::endl;

    if (bind(this->sockfd, (struct sockaddr *)&(this->my_addr), sizeof(this->my_addr)) < 0)
    {
        throw UDPSocketRuntimeError("Bind Failed.");
    }
}

/**
 * @brief Join to multicast group after serve port
 * 
 * @return true 
 * @return false 
 */
bool UDPSocket::joinMulticastGroup(std::string &addr)
{
    ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(addr.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (
        setsockopt(this->sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0 || 
        setsockopt(this->sockfd, IPPROTO_IP, IP_MULTICAST_IF, &mreq.imr_interface.s_addr, sizeof(mreq.imr_interface.s_addr)) < 0
    )
    {
        throw UDPSocketRuntimeError("Fail to join to multicast UDP");
    }

    return true;
}

/**
 * @brief Send a datagram to a remote host
 * 
 * @param d 
 * @param remote_ip 
 * @param remote_port
 */
int UDPSocket::sendData(Datagram &d, const std::string &remote_ip, const unsigned int remote_port)
{
    this->_pre_bind(&(this->remote_addr), remote_ip, remote_port);
    int r = sendto(this->sockfd, d.data(), d.size(), 0, (struct sockaddr *)&(this->remote_addr), sizeof(this->remote_addr));
    if (r == -1)
    {
        std::cerr << "Error sending message" << std::endl;
    }
    return r;
}

/**
 * @brief Send a datagram to a remote host by its name
 * 
 * @param d 
 * @param remote_ip 
 * @param remote_port 
 */
int UDPSocket::sendDataHostname(Datagram &d, const std::string &remote_name, const unsigned int remote_port)
{
    std::string ip = this->_resolve_hostname(remote_name);
    return this->sendData(d, ip, remote_port);
}

/**
 * @brief Receive a datagram
 * 
 * @param d Datagram
 * @param ip remote ip
 * @param port remote port
 * @return int length of received datagram
 */
int UDPSocket::receiveData(Datagram &d, std::string &ip, unsigned int *port)
{
    struct sockaddr_in recv_addr;
    socklen_t slen = sizeof(recv_addr);
    this->recv_len = recvfrom(this->sockfd, this->recv_buffer, this->BUF_LEN, 0, (struct sockaddr *)&recv_addr, &slen);
    if (this->recv_len > 0)
    {
        d.setdata(this->recv_buffer, this->recv_len);
        ip = std::string(inet_ntoa(recv_addr.sin_addr));
        *port = (unsigned int)recv_addr.sin_port;
    }

    return this->recv_len;
}

/**
 * @brief Return true if socket has incoming message
 * 
 * @return true 
 * @return false 
 */
bool UDPSocket::hasPendingData()
{
    fd_set rfds;
    // Wait 1 millisecond
    struct timeval tv = {0, 1000};
    FD_ZERO(&rfds);
    FD_SET(this->sockfd, &rfds);

    int retval = select(1, &rfds, NULL, NULL, &tv);

    if (retval == -1)
    {
        std::cerr << "Error on select socket" << std::endl;
    }
    else if (retval)
    {
        return true;
    }

    return false;
}