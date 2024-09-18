#include "TCPServer.h"

TCPServer::TCPServer(int port) : port(port), serverSocket(0), isRunning(false) {}

TCPServer::~TCPServer()
{
    stop();
}

void TCPServer::start()
{
    // Create socket
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        throw std::runtime_error("Failed to create socket");
    }
    // Prepare sockaddr_in structure
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(port);
    // Bind
    if (bind(serverSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        throw std::runtime_error("Bind failed");
    }
    // Listen

    listen(serverSocket, 1);
    isRunning = true;
    std::cout << "[TCPServer]:  Server started on port " << port << " Listening......" << std::endl;
    // Accept incoming connections
    struct sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);
    // Accept connection from an incoming client
    clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &clientAddrLen);
    char *ip = inet_ntoa(clientAddr.sin_addr); // 客户端IP地址转字符串
    unsigned short cport = ntohs(clientAddr.sin_port);
    printf("[TCPServer]:    Get new conncect success! client ip: %s, port is %d\n", ip, cport);
    if (clientSocket < 0)
    {
        throw std::runtime_error("Error accepting client connection");
    }
    handleClient(clientSocket);
    std::cout << "Server stopped" << std::endl;
}

void TCPServer::stop()
{
    isRunning = false;
    close(serverSocket); // Close the server socket
}

void TCPServer::parseJsonAndStore(const Json::Value &root, robot_joints &rob_joints)
{
    // Extract data from JSON and store into robot_joints structure
    // arg json: {"l":[11,12,13,14,15,16,17],"r":[21,22,23,24,25,26,27],"w":[4,5,6],"h":[1,2,3]}
    // 上位机和控制器走TCP/UDP通讯频率可能无法满足插补需求，
    // 可能上位机需要作连续插值，发送多条轨迹，在此函数中进行解析，并放在buffer中，再以一定频率下发到驱动器。

    Json::Value head = root["h"];
    Json::Value waist = root["w"];
    Json::Value left_arm = root["l"];
    Json::Value right_arm = root["r"];

    // Convert JSON arrays to floats and store in robot_joints struct
    for (int i = 0; i < 2; i++)
    {
        rob_joints.head[i] = head[i].asFloat();
    }

    for (int i = 0; i < 3; i++)
    {
        rob_joints.waist[i] = waist[i].asFloat();
    }

    for (int i = 0; i < 7; i++)
    {
        rob_joints.left_arm[i] = left_arm[i].asFloat();
        rob_joints.right_arm[i] = right_arm[i].asFloat();
    }
}

void TCPServer::handleClient(int clientSocket)
{

    while (isRunning)
    {
        // Receive data from client
        bytesRead = recv(clientSocket, recv_buf, sizeof(recv_buf), 0);
        if (bytesRead <= 0)
        {
            throw std::runtime_error("Received empty information from the client! ");
            break;
        }
        // Append received data to message
        message.append(recv_buf, bytesRead);
        std::cout << "[TCPServer]:  Received message from client: " << message << std::endl;
        if (!reader.parse(recv_buf, root))
        {
            std::cerr << "Failed to parse JSON" << std::endl;
            std::cerr << reader.getFormattedErrorMessages() << std::endl;
            close(clientSocket);
            break;
        }
        else
        {
            parseJsonAndStore(root, rob_joints);

            // Print the parsed values
            std::cout << "Parsed data:" << std::endl;
            std::cout << "head: " << rob_joints.head[0] << ", " << rob_joints.head[1] << std::endl;
            std::cout << "waist: " << rob_joints.waist[0] << ", " << rob_joints.waist[1] << ", " << rob_joints.waist[2] << std::endl;
            std::cout << "left_arm: ";
            for (int i = 0; i < 7; i++)
            {
                std::cout << rob_joints.left_arm[i] << " ";
            }
            std::cout << std::endl;
            std::cout << "right_arm: ";
            for (int i = 0; i < 7; i++)
            {
                std::cout << rob_joints.right_arm[i] << " ";
            }
            std::cout << std::endl;
        }
    }
    close(clientSocket);
    std::cout << "Client disconnected" << std::endl;
}
