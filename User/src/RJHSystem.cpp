#include <RJHSystem.hpp>

RJHSystem::RJHSystem() : logger(folderPath)
{
    motion = std::make_unique<Motion>(driver_name);
    running = true;
    parameter_server = ParameterServer::getInstance();
    udp_subscriber = new UdpSubscriber();
    udp_subscriber->recv_data(subscriber_rate);
    joints_publisher = std::make_unique<UdpPublisher>(address, 8012);
    status_publisher = std::make_unique<UdpPublisher>(address, 8011);
    sleep(2);
    // 获取当前角度初始化滤波器初始位置
}

RJHSystem::~RJHSystem()
{
    delete udp_subscriber;
    stop();
}

void RJHSystem::start()
{
    joints_publish_thread = std::thread(&RJHSystem::robot_info_publish, this, info_publish_rate);
    state_publish_thread = std::thread(&RJHSystem::robot_state_publish, this, state_publish_rate);
    recvRobotCmd_thread = std::thread(&RJHSystem::recvRobotCmd, this);
    RobotData::JointCmd _joint_cmd;
    RobotData::RobotCmd _robot_cmd;
    while (running)
    {
        parameter_server->getRobotInfo(_robot_cmd);
        parameter_server->getRobotInfo(_joint_cmd);
        // std::unique_lock<std::mutex> lock(robot_data_mutex);
        switch (_robot_cmd.motion_mode)
        {
        case SystemState::IDLE:
            idle();
            break;
        case SystemState::MANUAL:
            manual(_robot_cmd, _joint_cmd);
            break;
        case SystemState::MOVEL:
            movel(_robot_cmd, _joint_cmd);
            break;
        case SystemState::MOTION_CAPTURE:
            motion_capture(_robot_cmd, _joint_cmd);
            break;
        default:
            break;
        }
        usleep(1000);
    }
}

void RJHSystem::recvRobotCmd()
{
    /**
     * @brief 接收状态q
     * 从PC端接收命令更新机器人状态信息。更新motion_mode, enable， filter_enable
     */
    RobotData::RobotCmd _robot_cmd;
    RobotData::RobotState _newState;
    RobotData::JointCmd _joint_cmd;
    while (running)
    {
        parameter_server->getRobotInfo(_robot_cmd);
        parameter_server->getRobotInfo(_joint_cmd);
        // 判断运行状态命令
        if (_robot_cmd.running_mode == 1)
        {
            motion->motionStateSwitch(3);
        }
        // 判断使能命令
        if (_robot_cmd.enable == 1)
        {
            motion->enableRobot();
        }
        else
        {
            motion->diableRobot();
            motion->initFilterJoints();
        };
        // 判断复位命令
        if (_robot_cmd.reset_error == 1)
        {
            motion->resetMotionError();
            _newState.reset_feedback == 1;
            parameter_server->setRobotInfo(_newState);
        }
        else
        {
            _newState.reset_feedback == 1;
            parameter_server->setRobotInfo(_newState);
        };
        usleep(1000);
    }
}

void RJHSystem::stop()
{
    running = false;
    if (joints_publish_thread.joinable())
    {
        joints_publish_thread.join();
    }
    if (state_publish_thread.joinable())
    {
        state_publish_thread.join();
    }
    if (recvRobotCmd_thread.joinable())
    {
        recvRobotCmd_thread.join();
    }
}

void RJHSystem::robot_info_publish(int rate)
{
    try
    {
        RobotData::RobotPublishInfo publish_info;
        std::cout << "running robot_info_publish" << std::endl;
        while (running)
        {
            motion->getCurrentPosAndJoints(publish_info);
            parameter_server->setRobotInfo(publish_info);
            ssize_t sent_bytes_test = sendto(joints_publisher->socket_fd_,
                                             &publish_info, sizeof(RobotData::RobotPublishInfo), 0,
                                             (struct sockaddr *)&joints_publisher->server_addr_, joints_publisher->len);
            // std::ostringstream oss;
            // for (int i = 0; i < 7; ++i)
            // {
            //     oss << publish_info.joint_q_arm[0][i];
            //     if (i < 6)
            //     { // 在前六个元素后添加空格
            //         oss << ";";
            //     }
            // }
            std::string str_joint = oss.str();
            logger.logWrite("[current joints]: " + str_joint);
            if (sent_bytes_test < 0)
            {
                perror("sendto");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(rate));
        }
    }
    catch (const std::exception &e)
    {
        std::cout << "[UdpPublish Error]: Caught exception: " << e.what() << std::endl;
    }
    std::cout << "[UdpPublish]: stop running robot info publish" << std::endl;
}

void RJHSystem::robot_state_publish(int rate)
{
    std::cout << "running robot status publish" << std::endl;
    RobotData::RobotState robot_state_;
    while (running)
    {
        robot_state_.motion_mode = static_cast<int>(system_state);
        robot_state_.error = 0;
        robot_state_.running_mode = motion->getMotionState();
        ssize_t sent_bytes_test = sendto(status_publisher->socket_fd_,
                                         &robot_state_, sizeof(RobotData::RobotState), 0,
                                         (struct sockaddr *)&status_publisher->server_addr_, status_publisher->len);
        if (sent_bytes_test < 0)
        {
            perror("sendto error");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(rate));
    }
    std::cout << "[UdpPublish]: stop running robot status publish" << std::endl;
}

void RJHSystem::idle()
{
    system_state = SystemState::IDLE;
    motion->motionStateSwitch(1);
}

void RJHSystem::manual(RobotData::RobotCmd _robot_cmd, RobotData::JointCmd _joint_cmd)
{
    parameter_server->getRobotInfo(_robot_cmd);
    if (system_state == SystemState::IDLE)
    {
        system_state = SystemState::MANUAL;
        std::cout << "system state chang modle manual :" << std::endl;
    }
    else if (system_state == SystemState::MANUAL)
    {
        if (system_state == SystemState::MANUAL && _robot_cmd.running_mode == 2)
        {
            motion->motionStateSwitch(4);
            motion->robotMoveJoint(_joint_cmd);
        }
    }
    else
    {
        std::cout << "Cannot MANUAL. Current state: " << static_cast<int>(system_state) << std::endl;
    }
}

void RJHSystem::movel(RobotData::RobotCmd _robot_cmd, RobotData::JointCmd _joint_cmd)
{
    if (system_state == SystemState::IDLE)
    {
        system_state = SystemState::MOVEL;
        std::cout << "system state chang modle movel :" << std::endl;
    }
    else if (system_state == SystemState::MOVEL)
    {
        if (system_state == SystemState::MOVEL && _robot_cmd.running_mode == 2)
        {
            motion->motionStateSwitch(4);
            motion->robotMoveCartesion(_joint_cmd);
        }
    }
    else
    {
        std::cout << "Cannot MOTION_CAPTURE. Current state: " << static_cast<int>(system_state) << std::endl;
    }
}

void RJHSystem::motion_capture(RobotData::RobotCmd _robot_cmd, RobotData::JointCmd _joint_cmd)
{
    if (system_state == SystemState::IDLE)
    {
        system_state = SystemState::MOTION_CAPTURE;
        std::cout << "system state chang modle motion_capture :" << std::endl;
    }
    else if (system_state == SystemState::MOTION_CAPTURE)
    {
        if (system_state == SystemState::MOTION_CAPTURE && _robot_cmd.running_mode == 2)
        {
            motion->motionStateSwitch(4);
        }
        motion->robotMoveCartesion(_joint_cmd);
    }
    else
    {
        std::cout << "Cannot MOTION_CAPTURE. Current state: " << static_cast<int>(system_state) << std::endl;
    }
}

void RJHSystem::loadConfig()
{
    motion->loadMotionConfig();
}

int main(int argc, char *argv[])
{
    // //-----------------------user designation codes---------------
    std::cout << "hello rjh system" << std::endl;
    std::shared_ptr<RJHSystem> rjh_system_ptr = std::make_shared<RJHSystem>();
    rjh_system_ptr->start();
    //------------------------wait----------------------------------
    std::cout << "sys running" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));
    std::cout << "stop rjh system" << std::endl;
    // rjh_system_ptr->stop();
    return 0;
}
