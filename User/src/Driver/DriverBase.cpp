#include "DriverBase.h"

DriverBase::DriverBase()
{
}

DriverBase::~DriverBase()
{
    // 如果有需要，可以在这里进行资源释放
}
int DriverBase::enable_robot() const { return 0; }
int DriverBase::disable_robot() const { return 0; }
int DriverBase::get_robot_joints(RobotJoints &rob_joints) const { return 0; }
int DriverBase::set_robot_joints(RobotJoints &robot_joints) const { return 0; }
int DriverBase::set_cartesion(RobotJoints &robot_joints) const { return 0; }
int DriverBase::reset_driver_error() const { return 0;}
