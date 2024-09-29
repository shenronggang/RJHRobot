#ifndef MOTION_H
#define MOTION_H
#include <string>
#include "DriverBase.h"
#include "DriverFactory.hpp"
#include "LowFilter.h"
#include "RobotData.h"
#include "ik_7dof_ofst.h"
#include "rt_nonfinite.h"
#include "forward_kinematic_with_ofst.h"
#include <yaml-cpp/yaml.h>
#include <vector>
#define HEAD_DOF 2
#define WAIST_DOF 3
#define ARM_DOF 7
class Motion
{
private:
    std::unique_ptr<DriverBase> driver;
    std::string driver_name;
    std::thread _moveDriverThread;
    DriverBase::RobotJoints robot_current_joints, robot_current_pos,
        robot_move_joints, robot_move_cartesion;

    enum MotionState
    {
        INIT = 0,
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
    // 逆解参数
    int FOrB = 1;
    int LOrR = 1;
    int b_lt_or_rt = 1; // 0：左臂 1：右臂

    struct DH
    {
        double a_arr[ARM_DOF];
        double alpha_arr[ARM_DOF];
        double d_arr[ARM_DOF];
        double theta_arr[ARM_DOF];
    };
    struct Arm
    {
        DH dh;
        double joints_limit[ARM_DOF * 2];
    };
    struct MotionConfig
    {
        Arm left_arm;
        Arm right_arm;
    };
    MotionConfig motion_config;

public:
    Motion(const std::string &type);
    void loadMotionConfig(std::string path = "/home/robot/Work/system/robot_config/RJHRobot/User/config/blackrobot_config.yaml");
    ~Motion();
    void getFilterJoints(DriverBase::RobotJoints &robot_joint);
    void setFilterJoints(DriverBase::RobotJoints &robot_joint);
    void initFilterJoints();
    void filterEnable(bool enable);
    void enableRobot();
    void diableRobot();
    void getCurrentPosAndJoints(RobotData::RobotPublishInfo &robot_send_info_);
    void _moveDriver();
    void robotMoveJoint(RobotData::JointCmd &joint_cmd_);
    void robotMoveCartesion(RobotData::JointCmd &joint_cmd_);
    void motionStateSwitch(int running_mode);
    int getMotionState();
    void resetMotionError();
    void cartesion2Joints(DriverBase::RobotJoints &robot_current_joints, DriverBase::RobotJoints &robot_move_cartesion);
    void joints2Cartesion(DriverBase::RobotJoints &robot_current_joints, DriverBase::RobotJoints &robot_move_cartesion);
    void _ik(double *pos, bool l_or_r, double *ik_joint);
    void _fk(double *fk_joints, bool l_or_r, double *cart);
};

#endif // MOTION_H