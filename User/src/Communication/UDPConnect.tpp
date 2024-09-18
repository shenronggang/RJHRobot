#ifndef UDPCONNECT_TPP
#define UDPCONNECT_TPP

#include "UDPConnect.hpp"

template <typename T>
void UdpPublisher::sendmsg(const T& msg) {
    std::vector<char> buffer(sizeof(T));
    std::memcpy(buffer.data(), &msg, sizeof(T));
    ssize_t sent_len = sendto(socket_fd_, buffer.data(), buffer.size(), 0,
                              (struct sockaddr*)&server_addr_, sizeof(server_addr_));
    if (sent_len < 0) {
        throw std::runtime_error("sendto: " + std::string(strerror(errno)));
    }
}

template <typename T>
void UdpSubscriber::recvmsg(T& msg) {
    std::vector<char> buffer(sizeof(T));
    struct sockaddr_in from_addr;
    socklen_t from_addr_len = sizeof(from_addr);
    ssize_t recv_len = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                (struct sockaddr*)&from_addr, &from_addr_len);
    if (recv_len < 0) {
        throw std::runtime_error("recvfrom: " + std::string(strerror(errno)));
    }
    if (recv_len != sizeof(T)) {
        throw std::runtime_error("Received data size mismatch");
    }
    std::memcpy(&msg, buffer.data(), sizeof(T));
}

#endif // UDPCONNECT_TPP
