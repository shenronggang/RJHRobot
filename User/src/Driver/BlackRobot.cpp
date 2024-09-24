#include "BlackRobot.hpp"

BlackRobot::BlackRobot()
{
    std::cout << "=========================== create robot driver: BlackRobot===========================" << std::endl;
    int argc = 1;
    const char *fixed_path = "/home/robot/Work/system/robot_config/build/HYYRobotMain";
    char *argv[1];
    argv[0] = strdup(fixed_path);
    InitDriver(argc, argv);
    // 定时周期为1倍总线周期
    set_ec_cycle(ec_cycle);
    // TODO 总线设置异常
    ec_cur_cycle = get_ec_cycle();
    // if (cyc != ec_cycle)
    // {
    //     printf("[Error]: set ethercat cycle error, current cycle: %.8f\n", cyc);
    // }
    // else
    // {
    //     printf("Set ethercat cycle success: cyc: %.8f, ec_cycle: %.8f\n", cyc, ec_cycle);
    // }
    const char *robot_name = get_name_robot_device(get_deviceName(0, NULL), 0);
    std::cout << "robot_name: " << robot_name << std::endl;
    // 获取索要机器人名称
    robot_names = get_robot_names();
    std::cout << "robot_names Waist: " << robot_names.waist << std::endl;
    std::cout << "robot_names Left Hand: " << robot_names.left_arm << std::endl;
    std::cout << "robot_names Right Hand: " << robot_names.right_arm << std::endl;
    // 获取机器人关节数目
    robot_joint_num = get_joints_num();
    std::cout << "robot_joint_num Waist: " << robot_joint_num.waist_num << std::endl;
    std::cout << "robot_joint_num Left Hand: " << robot_joint_num.left_arm_num << std::endl;
    std::cout << "robot_joint_num Right Hand: " << robot_joint_num.right_arm_num << std::endl;
    reset_driver_error();
    std::cout << std::endl;
    disable_robot();
}

BlackRobot::~BlackRobot()
{
    disable_robot();
}

const BlackRobot::RobotNames BlackRobot::get_robot_names()
{
    /**
     * @brief 获取机器人所有组件名称
     *
     * @return RobotNames 机器人组件名称结构体
     */
    const char *device_name = get_deviceName(0, NULL);
    robot_names.head = get_name_additionaxis_device(device_name, 1);
    robot_names.waist = get_name_additionaxis_device(device_name, 0);
    robot_names.left_arm = get_name_robot_device(device_name, 0);
    robot_names.right_arm = get_name_robot_device(device_name, 1);
    return robot_names;
}

const BlackRobot::RobotJointsNum BlackRobot::get_joints_num()
{
    /**
     * @brief 获取机器人所有关节角数量
     *
     * @return  RobotJointsNum 机器人关节角数量结构体
     */
    robot_joint_num.head_num = get_group_dof(robot_names.head);
    robot_joint_num.waist_num = get_group_dof(robot_names.waist);
    robot_joint_num.left_arm_num = get_group_dof(robot_names.left_arm);
    robot_joint_num.right_arm_num = get_group_dof(robot_names.right_arm);
    return robot_joint_num;
}

int BlackRobot::get_robot_joints(RobotJoints &rob_joints) const
{
    /**
     * @brief 获取机器人所有关节角
     *
     * 获取机器人所有关节角，放入机器人结构体中。
     *
     * @param rob_joints 关节角结构体-角度
     */
    double head[2], waist[3], left_arm[7], right_arm[7];
    int res = 0;
    int res_head = GetGroupPosition(robot_names.head, head);
    for (int i = 0; i < 2; i++)
    {
        head[i] = radToDeg(head[i]);
        rob_joints.head[i] = static_cast<float>(head[i]);
    }
    int res_waist = GetGroupPosition(robot_names.waist, waist);
    for (int i = 0; i < 3; i++)
    {
        waist[i] = radToDeg(waist[i]);
        rob_joints.waist[i] = static_cast<float>(waist[i]);
    }
    int res_left_arm = GetGroupPosition(robot_names.left_arm, left_arm);
    for (int i = 0; i < 7; i++)
    {
        left_arm[i] = radToDeg(left_arm[i]);
        rob_joints.left_arm[i] = static_cast<float>(left_arm[i]);
    }
    int res_right_arm = GetGroupPosition(robot_names.right_arm, right_arm);
    for (int i = 0; i < 7; i++)
    {
        right_arm[i] = radToDeg(right_arm[i]);
        rob_joints.right_arm[i] = static_cast<float>(right_arm[i]);
    }
    if (res_head > 0)
        res |= (1 << 0);
    if (res_waist > 0)
        res |= (1 << 1);
    if (res_left_arm > 0)
        res |= (1 << 2);
    if (res_right_arm > 0)
        res |= (1 << 3);
    return res;
}

int BlackRobot::enable_robot() const
{
    /**
     * @brief 机器人上使能
     *
     * 机器人所有关节上使能。
     *
     */
    int result = 0, res;
    res = group_power_on(robot_names.head);
    if (res > 0)
        result |= (1 << 0);
    res = group_power_on(robot_names.waist);
    if (res > 0)
        result |= (1 << 1);
    res = group_power_on(robot_names.left_arm);
    if (res > 0)
        result |= (1 << 2);
    res = group_power_on(robot_names.right_arm);
    if (res > 0)
        result |= (1 << 3);
    return result;
}

int BlackRobot::disable_robot() const
{
    /**
     * @brief 机器人下使能
     *
     * 机器人所有关节下使能。
     *
     */
    int result = 0, res;
    res = group_power_off(robot_names.head);
    if (res > 0)
        result |= (1 << 0);
    res = group_power_off(robot_names.waist);
    if (res > 0)
        result |= (1 << 1);
    res = group_power_off(robot_names.left_arm);
    if (res > 0)
        result |= (1 << 2);
    res = group_power_off(robot_names.right_arm);
    if (res > 0)
        result |= (1 << 3);
    return result;
}

int BlackRobot::set_robot_joints(RobotJoints &robot_joints) const
{
    /**
     * @brief 下发关节角
     *
     * 函数使用时，需要先进行总线周期调用，根据总线周期下发关节角，使用时会判断是否经过过滤器。
     *
     * @param robot_joints 关节角结构体
     * @param filter_enable 是否启动过滤器
     * @return res 返回下发结果，全部异常返回1111
     */
    int res = 0;
    double head[2], waist[3], left_arm[7], right_arm[7];

    for (int i = 0; i < 2; i++)
    {
        head[i] = static_cast<double>(degToRad(robot_joints.head[i]));
    }
    for (int i = 0; i < 3; i++)
    {
        waist[i] = static_cast<double>(degToRad(robot_joints.waist[i]));
    }
    for (int i = 0; i < 7; i++)
    {
        left_arm[i] = static_cast<double>(degToRad(robot_joints.left_arm[i]));
    }
    for (int i = 0; i < 7; i++)
    {
        right_arm[i] = static_cast<double>(degToRad(robot_joints.right_arm[i]));
    }
    int res_head = SetGroupPosition(robot_names.head, head);
    int res_waist = SetGroupPosition(robot_names.waist, waist);
    int res_left_arm = SetGroupPosition(robot_names.left_arm, left_arm);
    int res_right_arm = SetGroupPosition(robot_names.right_arm, right_arm);
    if (res_head > 0)
        res |= (1 << 0);
    if (res_waist > 0)
        res |= (1 << 1);
    if (res_left_arm > 0)
        res |= (1 << 2);
    if (res_right_arm > 0)
        res |= (1 << 3);
    return res;
}

int BlackRobot::set_cartesion(RobotJoints &robot_joints) const
{
    /**
     * @brief 下发笛卡尔坐标系
     * 使用控制器默认逆解方式,先获取当前关节角,作为当前位置,再将输入笛卡尔位置进行逆解
     *
     * @param robot_joints 关节角结构体,左右手中存放的是xyz,rpy+臂形角
     * @return res 逆解失败返回-1,超过关节位置约束的0.8倍, 返回超限轴ID, 否则返回0
     */
    return 1;
}

int BlackRobot::ik(RobotJoints &cartesion, RobotJoints &robot_joint) const
{
    /**
     * @brief 逆运动学
     *
     * 使用控制器默认逆解方式,先获取当前关节角,作为当前位置,再将输入笛卡尔位置进行逆解
     *
     * @param cartesion 关节角结构体,左右手中存放的是xyz,rpy+臂形角
     * @param robot_joint 关节角结构体,逆解后得到的角度
     * @return res 逆解失败返回-1,超过关节位置约束的0.8倍, 返回超限轴ID, 否则返回0
     */
    robot_joint = cartesion;
    double left_arm_joint[10], right_arm_joint[10];
    R7_KINE left_arm_rkine, right_arm_rkine;
    double left_arm_xyz[3], right_arm_xyz[3];
    double left_arm_rpy[3], right_arm_rpy[3];
    double left_joint_ik[10], right_joint_ik[10];
    for (size_t i = 0; i < 3; i++)
    {
        left_arm_xyz[i] = static_cast<double>(cartesion.left_arm[i]);
        left_arm_rpy[i] = static_cast<double>(cartesion.left_arm[i + 3]);
        right_arm_xyz[3] = static_cast<double>(cartesion.right_arm[i]);
        right_arm_rpy[3] = static_cast<double>(cartesion.right_arm[i + 3]);
    }
    GetGroupPosition(robot_names.left_arm, left_arm_joint);                 // 获取左臂关节位置
    set_R7_KINE_joint(&left_arm_rkine, left_arm_joint);                     // 设置当前位置(用于选解)
    set_R7_KINE_pose(&left_arm_rkine, left_arm_xyz, left_arm_rpy);          // 设置待求目标
    int left_inv_ret = Kine_Inverse(robot_names.left_arm, &left_arm_rkine); // 逆运动学

    set_R7_KINE_joint(&right_arm_rkine, right_arm_joint);                      // 设置当前位置(用于选解)
    set_R7_KINE_pose(&right_arm_rkine, right_arm_xyz, right_arm_rpy);          // 设置待求目标
    GetGroupPosition(robot_names.right_arm, right_arm_joint);                  // 获取右臂关节位置
    int right_inv_ret = Kine_Inverse(robot_names.right_arm, &right_arm_rkine); // 逆运动学
    if (left_inv_ret != 0 && right_inv_ret != 0)
    {
        return -1;
    }
    get_R7_KINE_joint(&left_arm_rkine, left_joint_ik);   // 对应rkine.joint
    get_R7_KINE_joint(&right_arm_rkine, right_joint_ik); // 对应rkine.joint
    // 超过关节位置约束的0.8倍, 返回超限轴ID, 否则返回0
    int left_ret = IsRobotPositionLimitMargin(robot_names.left_arm, left_joint_ik, 0.8);
    if (left_ret != 0)
    {
        return left_ret;
    }
    int right_ret = IsRobotPositionLimitMargin(robot_names.right_arm, right_joint_ik, 0.8);
    if (right_ret != 0)
    {
        return right_ret;
    }
    for (size_t i = 0; i < 7; i++)
    {
        robot_joint.left_arm[i] = static_cast<double>(radToDeg(left_joint_ik[i]));
        robot_joint.right_arm[i] = static_cast<double>(radToDeg(right_joint_ik[i]));
    }
    return 0;
}

int BlackRobot::fk(RobotJoints &robot_joint, RobotJoints &cartesion) const
{

    R7_KINE left_rkine, right_rkine;
    int dof = 7;
    dof = robot_joint_num.left_arm_num;
    double left_arm_joint[dof], right_arm_joint[dof];
    for (int i = 0; i < dof; i++)
    {
        left_arm_joint[i] = (double)robot_joint.left_arm[i];
        right_arm_joint[i] = (double)robot_joint.right_arm[i];
    }
    double left_arm_xyz[3], left_arm_rpy[3];
    double right_arm_xyz[3], right_arm_rpy[3];

    init_R7_KINE2(&left_rkine, left_arm_joint, &dof, NULL, NULL);
    init_R7_KINE2(&right_rkine, right_arm_joint, &dof, NULL, NULL);
    int left_ret = Kine_Forward(robot_names.left_arm, &left_rkine); // 正运动学求解
    get_R7_KINE_pose(&left_rkine, left_arm_xyz, left_arm_rpy);

    init_R7_KINE2(&left_rkine, left_arm_joint, &dof, NULL, NULL);
    int right_ret = Kine_Forward(robot_names.right_arm, &right_rkine); // 正运动学求解
    if (left_ret != 0 || right_ret != 0)
    {
        return -1;
    }

    get_R7_KINE_pose(&left_rkine, right_arm_xyz, right_arm_rpy);
    for (size_t i = 0; i < 3; i++)
    {
        cartesion.left_arm[i] = left_arm_xyz[i];
        cartesion.left_arm[i + 3] = left_arm_rpy[i];
        cartesion.right_arm[i] = right_arm_xyz[i];
        cartesion.right_arm[i + 3] = right_arm_rpy[i];
    }

    return 0;
}

int BlackRobot::reset_driver_error() const
{
    clear_robot_move_error(0);
    clear_robot_move_error(1);
    clear_addition_move_error(0);
    clear_addition_move_error(1);
    clear_addition_driver_error(0);
    clear_addition_driver_error(1);
    clear_robot_driver_error(0);
    clear_robot_driver_error(1);
    return 0;
}
int BlackRobot::set_axis_joint(const char *robot_name, int axis_ID)
{
    // int axis_ID=1;//指定操作单轴id
    // RTimer timer;
    // double td = get_control_cycle(get_deviceName(0, NULL)); // 获取总线周期
    // axis_power_on(robot_name, axis_ID);                     // 单轴上电
    // double A = 1;
    // double f = 0.1;
    // double t = 0;
    // // 获取单轴的关节位置
    // double pos_base = GetAxisPosition(robot_name, axis_ID);
    // double pos_target = 0;
    // double pos_real = 0;
    // double pos_target_1 = 0;
    // while (robot_ok()) // 循环，当程序被强制停止或遇到错误时循环退出
    // {
    //     userTimer(&timer);                                     // 定时
    //     pos_target = A * cos(3.14 * 2 * f * t) - A + pos_base; // 设置期望数据
    //     // 获取当前单轴关节目标位置
    //     pos_target_1 = GetAxisTargetPosition(robot_name, axis_ID);
    //     pos_real = GetAxisPosition(robot_name, axis_ID);  // 获取当前单轴关节位置
    //     SetAxisPosition(robot_name, pos_target, axis_ID); // 设置单轴关节目标位置
    //     t += td;
    // }
    return 1;
}

void BlackRobot::InitDriver(int argc, char *argv[])
{
    int err1 = 0, err2 = 0;
    HYYRobotBase::command_arg arg;
    err1 = HYYRobotBase::commandLineParser(argc, argv, &arg);
    if (0 != err1)
    {
        std::cout << "！！！！！初始化总线异常" << std::endl;
    }
    err2 = HYYRobotBase::system_initialize(&arg);
    if (0 != err2)
    {
        std::cout << "！！！！！初始化系统异常" << std::endl;
    }
}
void BlackRobot::run_test()
{
    // int axis_ID = 1;                          // 指定操作单轴id
    // axis_power_on(robot_names.head, axis_ID); // 单轴上电
    // double A = 1;
    // double f = -0.1;
    // double t = 0;
    // // 获取单轴的关节位置
    // double pos_base = GetAxisPosition(robot_names.head, axis_ID);

    // double pos_target = 0;
    // double pos_real = 0;
    // double pos_target_1 = 0;
    // double end = degToRad(radToDeg(pos_base) + 90);
    // double x = 10;
    // double td = get_control_cycle(get_deviceName(0, NULL)); // 获取总线周期
    // while (robot_ok() and x > 1)                            // 循环，当程序被强制停止或遇到错误时循环退出
    // {
    //     userTimer(&timer);                                     // 定时
    //     pos_target = A * cos(3.14 * 2 * f * t) + A + pos_base; // 设置期望数据
    //     // 获取当前单轴关节目标位置
    //     pos_target_1 = GetAxisTargetPosition(robot_names.head, axis_ID);
    //     pos_real = GetAxisPosition(robot_names.head, axis_ID);  // 获取当前单轴关节位置
    //     SetAxisPosition(robot_names.head, pos_target, axis_ID); // 设置单轴关节目标位置
    //     double tmp[2] = {pos_target_1, pos_real};
    //     // 将运行过程中的数据保存到servo_data.txt中，可用于分析
    //     //  RSaveDataFast1("servo_data",1,100,2,tmp);
    //     t += td;
    //     x = abs(radToDeg(end - pos_real));
    // }
    // double t = 1;
    // double pos_target_1 = 0;
    // int res = enable_robot();
    // // disable_robot();
    // RobotJoints new_robot_joints;
    // RobotJoints current_robj;
    // int res_setjoints;
    // userTimer(&timer); // 定时
    // while (robot_ok())
    // {
    //     pos_target_1 = GetAxisTargetPosition(robot_names.head, 0);
    //     // new_robot_joints.left_arm[1] = degToRad(radToDeg(robot_joints.left_arm[1]) + 10);
    //     get_robot_joints(current_robj);
    //     new_robot_joints = current_robj;
    //     new_robot_joints.head[1] = current_robj.head[1] + 0.1;
    //     // for (int i = 0; i < robot_joint_num.waist_num; i++)
    //     // {
    //     //     new_robot_joints.waist[i] = current_robj.waist[i]+0.1;
    //     // }
    //     // for (int i = 0; i < robot_joint_num.left_arm_num; i++)
    //     // {
    //     //     new_robot_joints.left_arm[i] = current_robj.left_arm[i]+0.1;
    //     // }
    //     // for (int i = 0; i < robot_joint_num.right_arm_num; i++)
    //     // {
    //     //     new_robot_joints.right_arm[i] = current_robj.right_arm[i]+0.1;
    //     // }
    //     res_setjoints = set_robot_joints(new_robot_joints);

    //     t += ec_cur_cycle;
    // }
}
