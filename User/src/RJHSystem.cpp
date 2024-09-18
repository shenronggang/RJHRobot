#include <RJHSystem.hpp>

RJHSystem::RJHSystem()
{
    motion = std::make_unique<Motion>(driver_name);
    running = true;
    robot_data = new RobotData();
    udp_subscriber = new UdpSubscriber(robot_data);
    udp_subscriber->recv_data(subscriber_rate);
    joints_publisher = std::make_unique<UdpPublisher>(address, 8012);
    status_publisher = std::make_unique<UdpPublisher>(address, 8011);
    sleep(2);
    // 获取当前角度初始化滤波器初始位置
}

RJHSystem::~RJHSystem()
{
    delete robot_data;

    stop();
}

void RJHSystem::start()
{
    joints_publish_thread = std::thread(&RJHSystem::robot_info_publish, this, info_publish_rate);
    state_publish_thread = std::thread(&RJHSystem::robot_state_publish, this, state_publish_rate);
    recvRobotCmd_thread = std::thread(&RJHSystem::recvRobotCmd, this);
    // system_state_update_thread = std::thread(&RJHSystem::system_state_update, this);
    while (running)
    {
        std::unique_lock<std::mutex> lock(robot_data_mutex);
        switch (robot_data->robot_info_.robot_cmd_.motion_mode)
        {
        case SystemState::IDLE:
            idle();
            break;
        case SystemState::MANUAL:
            manual();
            break;
        case SystemState::MOTION_CAPTURE:
            motion_capture();
            break;
        default:
            break;
        }
        lock.unlock();
        usleep(1000);
    }
}

void RJHSystem::system_state_update()
{
}
void RJHSystem::recvRobotCmd()
{
    /**
     * @brief 接收状态q
     * 从PC端接收命令更新机器人状态信息。更新motion_mode, enable， filter_enable
     */
    while (running)
    {
        std::unique_lock<std::mutex> lock(robot_data_mutex);
        // system_state = static_cast<SystemState>(robot_data->robot_info_.robot_cmd_.motion_mode);
        // motion->filterEnable(robot_data->robot_info_.robot_cmd_.filter_enable);
        if (robot_data->robot_info_.robot_cmd_.running_mode == 1)
        {
            motion->motionStateSwitch(3);
        }

        if (robot_data->robot_info_.robot_cmd_.enable == 1)
        {
            motion->enableRobot();
        }
        else
        {
            motion->diableRobot();
        };
        lock.unlock();
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

void RJHSystem::_update_joints()
{
    std::lock_guard<std::mutex> lock(robot_data_mutex);
    // TODO 更新机器人位姿
    motion->getCurrentJoints(robot_data->robot_info_);
}

void RJHSystem::_update_robot_state()
{
    std::lock_guard<std::mutex> lock(robot_data_mutex);
    robot_data->robot_info_.robot_state_.motion_mode = static_cast<int>(system_state);
    robot_data->robot_info_.robot_state_.error = 0;
    robot_data->robot_info_.robot_state_.running_mode = motion->getMotionState();
}

void RJHSystem::idle()
{
    system_state = SystemState::IDLE;
    motion->motionStateSwitch(1);
}

void RJHSystem::motion_capture()
{
    if (system_state == SystemState::IDLE)
    {
        system_state = SystemState::MOTION_CAPTURE;
        std::cout << "system state chang modle manual :" << std::endl;
    }
    else if (system_state == SystemState::MOTION_CAPTURE)
    {
        if (system_state == SystemState::MOTION_CAPTURE && robot_data->robot_info_.robot_cmd_.running_mode == 2)
        {
            motion->motionStateSwitch(4);
        }
        // motion->motionStateSwitch(robot_data->robot_info_.robot_cmd_.running_mode);
        motion->robotMoveCartesion(robot_data->robot_info_);
    }
    else
    {
        std::cout << "Cannot MOTION_CAPTURE. Current state: " << static_cast<int>(system_state) << std::endl;
    }
}

void RJHSystem::manual()
{
    if (system_state == SystemState::IDLE)
    {
        system_state = SystemState::MANUAL;
        std::cout << "system state chang modle manual :" << std::endl;
    }
    else if (system_state == SystemState::MANUAL)
    {
        if (system_state == SystemState::MANUAL && robot_data->robot_info_.robot_cmd_.running_mode == 2)
        {
            motion->motionStateSwitch(4);
        }
        // motion->motionStateSwitch(robot_data->robot_info_.robot_cmd_.running_mode);
        motion->robotMoveJoint(robot_data->robot_info_);
    }
    else
    {
        std::cout << "Cannot MANUAL. Current state: " << static_cast<int>(system_state) << std::endl;
    }
}

void RJHSystem::robot_info_publish(int rate)
{
    try
    {
        std::cout << "running robot_info_publish" << std::endl;
        while (running)
        {
            _update_joints();
            ssize_t sent_bytes_test = sendto(joints_publisher->socket_fd_,
                                             &robot_data->robot_info_.robot_send_info_, sizeof(RobotData::RobotPublishInfo), 0,
                                             (struct sockaddr *)&joints_publisher->server_addr_, joints_publisher->len);
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
    while (running)
    {
        _update_robot_state();
        ssize_t sent_bytes_test = sendto(status_publisher->socket_fd_,
                                         &robot_data->robot_info_.robot_state_, sizeof(RobotData::RobotState), 0,
                                         (struct sockaddr *)&status_publisher->server_addr_, status_publisher->len);
        if (sent_bytes_test < 0)
        {
            perror("sendto");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(rate));
    }
    std::cout << "[UdpPublish]: stop running robot status publish" << std::endl;
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
