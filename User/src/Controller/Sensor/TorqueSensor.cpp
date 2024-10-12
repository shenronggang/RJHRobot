#include <TorqueSensor.hpp>

TorqueSensor::TorqueSensor(/* args */)
{
    torque_name.left_torque = "left";
    torque_name.right_torque = "right";

    int ret = CreateTorqueSensor(torque_name.left_torque);
    printf("CreateTorqueSensor,ret=%d\n", ret);
    ret = CreateTorqueSensor(torque_name.right_torque);
    printf("CreateTorqueSensor,ret=%d\n", ret);
}

TorqueSensor::~TorqueSensor()
{
}

void TorqueSensor::get_torque(double (&torque)[2][6])
{
    double left_torque[6], right_torque[6];
    int ret_left = GetSensorTorque(torque_name.left_torque, left_torque);
    if (ret_left != 0)
    {
        printf("Get left torque failed\n");
    }
    memcpy(torque[0], left_torque, sizeof(left_torque));

    int ret_right = GetSensorTorque(torque_name.right_torque, right_torque);
    if (ret_right != 0)
    {
        printf("Get right torque failed\n");
    }
    memcpy(torque[1], right_torque, sizeof(right_torque));
}
void TorqueSensor::get_torque(double *torque, const char *torque_name)
{
    int ret_left = GetSensorTorque(torque_name, torque);
    if (ret_left != 0)
    {
        printf("Get left torque failed\n");
    }}
void TorqueSensor::clean_torque(int num)
{
    int ret = 0;
    if (num == 0)
    {
        ret = TorqueSensorOpenBias(torque_name.left_torque);
    }
    else
    {
        ret = TorqueSensorOpenBias(torque_name.right_torque);
    }
    if (ret != 0)
    {
        printf("Clear torque failed\n");
    }
    if (num == 0)
    {
        ret = TorqueSensorCloseBias(torque_name.left_torque);
    }
    else
    {
        ret = TorqueSensorCloseBias(torque_name.right_torque);
    }
}