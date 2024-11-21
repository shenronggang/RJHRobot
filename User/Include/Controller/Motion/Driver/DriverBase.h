#ifndef DRIVERBASE_H
#define DRIVERBASE_H

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cstring>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>

class DriverBase
{
protected:
    mutable std::mutex ec_mtx;
public:
    struct RobotJoints
    {
        float head[2];
        float waist[3];
        float left_arm[7];
        float right_arm[7];
        float left_leg[6];
        float right_leg[6];
    };

    DriverBase();
    virtual ~DriverBase();

    virtual int enable_robot() const = 0;
    virtual int disable_robot() const = 0;
    virtual int get_robot_joints(RobotJoints &rob_joints) const;
    virtual int set_robot_joints(RobotJoints &robot_joints) const;
    virtual int set_cartesion(RobotJoints &robot_joints) const;
    virtual int reset_driver_error() const;
    virtual void run_test() = 0;
    template <typename T>
    T degToRad(T degrees) const
    {
        return degrees * (static_cast<T>(M_PI) / 180.0);
    }
    template <typename T>
    T radToDeg(T radians) const
    {
        return radians * (180.0 / static_cast<T>(M_PI));
    }
};
using DriverBasePtr = std::unique_ptr<DriverBase>;

#endif // DRIVERBASE_H