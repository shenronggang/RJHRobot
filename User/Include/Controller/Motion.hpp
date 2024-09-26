#ifndef MOTION_H
#define MOTION_H
#include <string>
#include "DriverBase.h"
#include "DriverFactory.hpp"
#include "LowFilter.h"
#include "RobotData.h"
#include "ik_7dof_ofst.h"
#include "rt_nonfinite.h"

class Motion
{
private:
    std::unique_ptr<DriverBase> driver;
    std::string driver_name;
    std::thread _moveDriverThread;
    DriverBase::RobotJoints robot_current_joints;
    DriverBase::RobotJoints robot_move_joints;
    enum MotionState
    {
        INIT=0,
        INIT_OK,
        READY,
        READY_OK,
        RUN,
        STOP,
        DISABLE,
        ERROR
    };
    MotionState motion_state = MotionState::INIT;
    LowFilter head_filters[2];
    LowFilter waist_filters[3];
    LowFilter left_arm_filters[7];
    LowFilter right_arm_filters[7];
    std::mutex filter_mtx;
    std::mutex moving_mtx;
    std::atomic<bool> filter_enable;
    DriverBase::RobotJoints robot_joints_filtered;
    bool motor_on = false;
    bool motor_running = true;

public:
    Motion(const std::string &type);
    ~Motion();
    void getFilterJoints(DriverBase::RobotJoints &robot_joint);
    void setFilterJoints(DriverBase::RobotJoints &robot_joint);
    void initFilterJoints();
    void filterEnable(bool enable);
    void enableRobot();
    void diableRobot();
    void getCurrentJoints(RobotData::RobotInfo &robot_info_);
    void _moveDriver();
    void robotMoveJoint(RobotData::RobotInfo &robot_info_);
    void robotMoveCartesion(RobotData::RobotInfo &robot_info_);
    void motionStateSwitch(int running_mode);
    int getMotionState();
    void resetMotionError();
};

#endif // MOTION_H