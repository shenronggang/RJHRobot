#include "UDPConnect.hpp"

// UdpPublisher implementation
UdpPublisher::UdpPublisher(const std::string &address, unsigned short port)
    : address_(address), port_(port)
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0)
    {
        throw std::runtime_error("socket: " + std::string(strerror(errno)));
    }
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    if (inet_pton(AF_INET, address_.c_str(), &server_addr_.sin_addr) <= 0)
    {
        throw std::runtime_error("inet_pton: " + std::string(strerror(errno)));
    }

    len = sizeof(server_addr_);
    std::cout << "[UDPPublish]: creat success!!! address: " << address_ << " port: " << port_ << std::endl;
}

UdpSubscriber::UdpSubscriber()
{
    for (int i = 0; i < PORT_NUM; i++)
    {
        server_socket[i] = new Server_Socket(SERVER_PORT[i]);
    }
}

void UdpSubscriber::recv_data(int subscriber_rate)
{
    Thread = std::thread(&UdpSubscriber::recv_func, this, subscriber_rate);
    recv_ing = true;
}

void UdpSubscriber::recv_func(int subscriber_rate)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    while (recv_ing)
    {
        for (int i = 0; i < PORT_NUM; i++)
        {
            int cn = server_socket[i]->recv_fun(recv_buf);
            if (cn > 0)
            {
                // if(server_socket[i]->_port == PORT_OCU) printf("###cn = %d\n",cn);
                // printf("port = %d\n", server_socket[i]->_port);
                rcv_calback(recv_buf, server_socket[i]->_port);
                fail_nn = 0;
            }
            else
            {
                fail_nn++;
                if (fail_nn >= 5000)
                {
                    fail_nn = 5000;
                }
            }
        }
        usleep(subscriber_rate);
    }
}

void UdpSubscriber::udpclose()
{
    for (int i = 0; i < 2; i++)
    {
        server_socket[i]->socket_close();
    }
}

int UdpSubscriber::rcv_calback(void *package, uint16_t port)
{
    char *buf = (char *)package;

    // bool data_error = false;
    switch (port)
    {
    case PORT_JOINTS:
        if (robot_cmd_.motion_mode == 2)
        {
            memcpy(&joint_cmd_, buf, sizeof(RobotData::JointCmd));
            parameter_server->setRobotInfo(joint_cmd_);

            break;
        }
    case PORT_CARTESION:
        if (robot_cmd_.motion_mode == 5)
        {
            memcpy(&joint_cmd_, buf, sizeof(RobotData::JointCmd));
            parameter_server->setRobotInfo(joint_cmd_);

            break;
        }
        else if (robot_cmd_.motion_mode == 4)
        {
            memcpy(&joint_cmd_, buf, sizeof(RobotData::JointCmd));
            parameter_server->setRobotInfo(joint_cmd_);

            break;
        }
    case PORT_ROBOT_CMD:
        memcpy(&robot_cmd_, buf, sizeof(RobotData::RobotCmd));
    parameter_server->setRobotInfo(robot_cmd_);
        break;
    default:
        break;
    }
    
    return 0;
}
