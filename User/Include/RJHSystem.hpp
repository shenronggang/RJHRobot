#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdio.h>
#include <iostream>
#include <memory>
#include <atomic>
#include <mutex>
#include <UDPConnect.hpp>
#include <RobotData.h>
#include <HYYRobotInterface.h>
#include <Motion.hpp>

class RJHSystem
{
private:
    string driver_name = "BRobot";
    std::unique_ptr<Motion> motion;

    // std::string address = "192.168.112.62";  //sunyusheng
    std::string address = "192.168.113.209"; //chenchensheng
    // std::string address = "192.168.113.235"; //jiangzhengjie
    // std::string address = "192.168.113.211"; //lizihan
    RobotData *robot_data;

    UdpSubscriber *udp_subscriber;

    std::atomic<bool> running;
    std::mutex robot_data_mutex;
    std::unique_ptr<UdpPublisher> joints_publisher;
    std::unique_ptr<UdpPublisher> status_publisher;
    std::thread joints_publish_thread;
    std::thread state_publish_thread;
    std::thread recvRobotCmd_thread;
    std::thread system_state_update_thread;

    int info_publish_rate = 500;
    int state_publish_rate = 500;
    int subscriber_rate = 2;
    void _update_joints();
    void _update_robot_state();
    void robot_info_publish(int rate);
    void robot_state_publish(int rate);
    int dof_head = 2;
    int dof_waist = 3;
    int dof_left_arm = 7;
    int dof_right_arm = 7;
    int dof_left_leg = 6;
    int dof_right_leg = 6;
    enum SystemState
    {
        IDLE = 0,
        DYNAMIC,
        MANUAL,
        AUTO,
        DEMONSTRATOR,
        MOTION_CAPTURE
    };
    SystemState system_state = SystemState::IDLE;
    void idle();
    void manual();
    void motion_capture();

    void format_joints(DriverBase::RobotJoints &robot_joints);
    DriverBase::RobotJoints format_joints();

public:
    RJHSystem();
    ~RJHSystem();
    void start();
    void stop();
    void recvRobotCmd();
    void system_state_update();
    void testMotion();
};
#endif // SYSTEM_H