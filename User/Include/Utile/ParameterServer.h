#ifndef PARAMETERSERVER_H
#define PARAMETERSERVER_H

#include <memory>
#include <mutex>
#include "RobotData.h" // 假设 RobotData 定义在这个头文件中

class ParameterServer
{
public:
    ParameterServer(ParameterServer &) = delete;
    ParameterServer &operator=(ParameterServer &) = delete;
    static std::shared_ptr<ParameterServer> getInstance();
    RobotData::RobotInfo getRobotInfo();

    void getRobotInfo(RobotData::RobotInfo &info);
    void getRobotInfo(RobotData::RobotState &robot_state);
    void getRobotInfo(RobotData::JointCmd &joint_cmd);
    void getRobotInfo(RobotData::RobotCmd &robot_cmd);


    void setRobotInfo(RobotData::RobotInfo &newInfo);
    void setRobotInfo(RobotData::RobotState &newState);
    void setRobotInfo(RobotData::RobotPublishInfo &newSendInfo);
    void setRobotInfo(RobotData::RobotCmd &newCmd);
    void setRobotInfo(RobotData::JointCmd &newJointCmd);

private:
    static std::shared_ptr<ParameterServer> instance_;
    static std::mutex mtx_; // 用于保护实例创建
    RobotData robot_data_;
    ParameterServer();

    static std::shared_ptr<ParameterServer> instance;
    static std::once_flag initInstanceFlag;

    std::shared_ptr<RobotData::RobotInfo> robot_info_;
    std::mutex dataMutex_;
};

#endif // PARAMETERSERVER_H
