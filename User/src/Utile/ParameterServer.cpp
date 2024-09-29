#include "ParameterServer.h"

// 初始化静态成员
std::shared_ptr<ParameterServer> ParameterServer::instance_ = nullptr;
std::once_flag ParameterServer::initInstanceFlag; // 定义一次性标志

// 构造函数实现
ParameterServer::ParameterServer() {}

// 获取单例实例的方法
std::shared_ptr<ParameterServer> ParameterServer::getInstance() {
    std::call_once(initInstanceFlag, []() {
        instance_ = std::shared_ptr<ParameterServer>(new ParameterServer());
    });
    return instance_;
}

// 获取机器人状态
void ParameterServer::getRobotInfo(RobotData::RobotState &robot_state)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    robot_state =  robot_data_.robot_info_.robot_state_; // 复制当前状态
}

// 获取机器人指令
void ParameterServer::getRobotInfo(RobotData::RobotCmd &robot_cmd)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    robot_cmd =  robot_data_.robot_info_.robot_cmd_; // 复制当前关节指令
}
// 获取关节指令
void ParameterServer::getRobotInfo(RobotData::JointCmd &joint_cmd)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    joint_cmd =  robot_data_.robot_info_.joint_cmd_; // 复制当前关节指令
}

// 设置机器人信息
void ParameterServer::getRobotInfo(RobotData::RobotInfo &newInfo)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    newInfo = robot_data_.robot_info_; // 直接修改指针指向的对象
}

// 设置机器人信息
void ParameterServer::setRobotInfo(RobotData::RobotInfo &newInfo)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    robot_data_.robot_info_= newInfo; // 直接修改指针指向的对象
}

// 设置机器人状态
void ParameterServer::setRobotInfo(RobotData::RobotState &newState)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
     robot_data_.robot_info_.robot_state_ = newState; // 修改机器人状态
}

// 设置发送信息
void ParameterServer::setRobotInfo(RobotData::RobotPublishInfo &newSendInfo)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    robot_data_.robot_info_.robot_send_info_ = newSendInfo; // 修改发送信息
}

// 设置机器人指令
void ParameterServer::setRobotInfo(RobotData::RobotCmd &newCmd)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
     robot_data_.robot_info_.robot_cmd_ = newCmd; // 修改指令
}

// 设置关节指令
void ParameterServer::setRobotInfo(RobotData::JointCmd &newJointCmd)
{
    std::lock_guard<std::mutex> lock(dataMutex_);
    robot_data_.robot_info_.joint_cmd_ = newJointCmd; // 修改关节指令
}
