#ifndef TORQUESENSOR_H
#define TORQUESENSOR_H

#include <string>
#include "HYYRobotInterface.h"
using namespace HYYRobotBase;
using namespace std;

class TorqueSensor
{
private:
    /* data */
public:
    struct TorqueName
    {
        const char* left_torque;
        const char* right_torque;
    };
    TorqueName torque_name;
    TorqueSensor(/* args */);
    ~TorqueSensor();

    void get_torque(double *torque, const char *torque_name);
    void get_torque(double (&torque)[2][6]);
    void clean_torque(int num);
};

#endif