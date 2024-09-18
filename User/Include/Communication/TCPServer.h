#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

// For socket programming
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include <stdexcept>

#include <json/json.h>
#include <json/json-forwards.h>


// socket tcp server
class TCPServer
{
public:
    struct robot_joints
    {
        float head[2];
        float waist[3];
        float left_arm[7];
        float right_arm[7];
    };

    robot_joints rob_joints = {
        {0.0f, 0.0f},                               // head
        {0.0f, 0.0f, 0.0f},                         // waist
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}, // left_arm
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}  // right_arm
    };
    TCPServer(int port);
    ~TCPServer();

    void start();
    
    robot_joints RecvJointsFromClient(){return rob_joints;}
    void stop();

private:
    int port;
    int serverSocket;
    Json::Value root;
    Json::Reader reader;
    char recv_buf[1024];
    std::string message;
    ssize_t bytesRead;
    std::atomic<bool> isRunning;
    int clientSocket;
    void handleClient(int clientSocket);
    void parseJsonAndStore(const Json::Value &root, robot_joints &rob_joints);
    void run_test();
};

