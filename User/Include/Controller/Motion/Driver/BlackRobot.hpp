#ifndef BLACKROBOT_H
#define BLACKROBOT_H
#include <HYYRobotInterface.h>
#include <DriverBase.h>
#include <memory>
#include <string>
#include <iostream>
#include <cmath>

// #pragma once

using namespace HYYRobotBase;

class BlackRobot : public DriverBase
{
private:
    struct RobotNames
    {
        const char *head;
        const char *waist;
        const char *left_arm;
        const char *right_arm;
    };
    struct RobotJointsNum
    {
        int head_num;
        int waist_num;
        int left_arm_num;
        int right_arm_num;
    };

    RTimer timer;
    const char *device_name;
    double ec_cycle = 2;
    double ec_cur_cycle = 0;
    RobotNames robot_names;
    RobotJointsNum robot_joint_num;
    
public:
    void InitDriver(int argc, char *argv[]);
    BlackRobot();
    inline void set_ec_cycle(double cycle)
    {
        initUserTimer(&timer, 0, cycle);
        ec_cycle = cycle;
    };
    inline double get_ec_cycle()
    {
        double cyc = get_control_cycle(get_deviceName(0, NULL));
        return cyc;
    }
    const RobotNames get_robot_names();
    const RobotJointsNum get_joints_num();
    int enable_robot() const override;
    int disable_robot() const override;
    int get_robot_joints(RobotJoints &rob_joints) const override;
    int set_robot_joints(RobotJoints &robot_joints) const override;
    int set_cartesion(RobotJoints &robot_joints) const override;
    int reset_driver_error() const override;
    void run_test();
    int set_axis_joint(const char *robot_name, int axis_ID = 1);
    virtual ~BlackRobot();
};

#endif // BLACKROBOT_H