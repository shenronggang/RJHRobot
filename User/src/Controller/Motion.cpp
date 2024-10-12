#include "Motion.hpp"

// TODO 将driver放入Motion中维护
Motion::Motion(const std::string &driver_name) : driver_name(driver_name), filter_enable(true)
{
    loadMotionConfig();
    memset(&robot_joints_filtered, 0, sizeof(robot_joints_filtered));
    driver = DriverFactory::createDriver(driver_name);
    motion_state = MotionState::INIT;
    // 获取所有关节位置
    int res_ = driver->get_robot_joints(robot_current_joints);
    std::cout << "robot current joints: " << res_ << std::endl;
    std::cout << "waist: ";
    for (int i = 0; i < 3; i++)
    {
        std::cout << " " << robot_current_joints.waist[i];
    }
    std::cout << std::endl;
    std::cout << "left arm: ";
    for (int i = 0; i < 7; i++)
    {
        std::cout << " " << robot_current_joints.left_arm[i];
    }
    std::cout << std::endl;
    std::cout << "right arm: ";
    for (int i = 0; i < 7; i++)
    {
        std::cout << " " << robot_current_joints.right_arm[i];
    }
    // 持续下发关节角给驱动器

    initFilterJoints();
    getFilterJoints(robot_joints_filtered);
    _moveDriverThread = std::thread(&Motion::_moveDriver, this);
    motion_state = MotionState::INIT_OK;
    motor_running = true;
}

void Motion::_moveDriver()
{
    while (motor_running)
    {
        getFilterJoints(robot_joints_filtered);
        if (motion_state == MotionState::RUN && motor_on == 1)
        {
            if (filter_enable)
            {
                int res = driver->set_robot_joints(robot_joints_filtered);
                if (res != 0)
                {
                    printf("set move joints to ethercat error: %d", res);
                }
            }
        }
        else
        {
        }
        usleep(1000);
    }
    std::cout << "[Motion]: Driver running error!!!!" << std::endl;
}

void loadMotionConfig()
{
}

void Motion::motionStateSwitch(int running_mode)
{
    motion_state = static_cast<MotionState>(running_mode);
}

int Motion::getMotionState()
{
    int s = static_cast<int>(motion_state);
    return s;
}

void Motion::filterEnable(bool enable)
{
    filter_enable.store(enable);
}
void Motion::getFilterJoints(DriverBase::RobotJoints &robot_joint)
{
    std::lock_guard<std::mutex> lock(filter_mtx);
    for (int i = 0; i < 2; i++)
    {
        robot_joints_filtered.head[i] = head_filters[i].get();
        robot_joint.head[i] = robot_joints_filtered.head[i];
    }
    for (int i = 0; i < 3; i++)
    {
        robot_joints_filtered.waist[i] = waist_filters[i].get();
        robot_joint.waist[i] = robot_joints_filtered.waist[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_joints_filtered.left_arm[i] = left_arm_filters[i].get();
        robot_joint.left_arm[i] = robot_joints_filtered.left_arm[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_joints_filtered.right_arm[i] = right_arm_filters[i].get();
        robot_joint.right_arm[i] = robot_joints_filtered.right_arm[i];
    }
}

void Motion::setFilterJoints(DriverBase::RobotJoints &robot_joint)
{
    std::lock_guard<std::mutex> lock(filter_mtx);
    for (int i = 0; i < 2; i++)
    {
        head_filters[i].update(robot_joint.head[i]);
    }
    for (int i = 0; i < 3; i++)
    {
        waist_filters[i].update(robot_joint.waist[i]);
    }
    for (int i = 0; i < 7; i++)
    {
        left_arm_filters[i].update(robot_joint.left_arm[i]);
        right_arm_filters[i].update(robot_joint.right_arm[i]);
    }
}

void Motion::initFilterJoints()
{
    driver->get_robot_joints(robot_current_joints);
    for (int i = 0; i < 2; i++)
    {
        head_filters[i].init(robot_current_joints.head[i]);
    }
    for (int i = 0; i < 3; i++)
    {
        waist_filters[i].init(robot_current_joints.waist[i]);
    }
    for (int i = 0; i < 7; i++)
    {
        left_arm_filters[i].init(robot_current_joints.left_arm[i]);
        right_arm_filters[i].init(robot_current_joints.right_arm[i]);
    }
}

void Motion::enableRobot()
{
    if (motor_on == false)
    {
        int res = driver->enable_robot();
        // if (res == 0)
        // {
        motor_on = true;
        // }
        // else
        // {
        //     motor_on == false;
        //     printf("enable robot error: %d", res);
        // }
    }
}

void Motion::diableRobot()
{
    if (motor_on == true)
    {
        int res = driver->disable_robot();
        // if (res == 0)
        // {
        motor_on = false;
        // }
        // else
        // {
        //     motor_on = true;
        //     printf("disable robot error: %d", res);
        // }
    }
}

void Motion::getCurrentPosAndJoints(RobotData::RobotPublishInfo &robot_send_info_)
{
    int res = driver->get_robot_joints(robot_current_joints);
    joints2Cartesion(robot_current_joints, robot_current_pos);
    if (res != 0)
    {
        printf("get current joints form ethercat error: %d", res);
    }
    for (int i = 0; i < HEAD_DOF; i++)
    {
        robot_send_info_.joint_q_head[i] = robot_current_joints.head[i];
        robot_send_info_.joint_q_head_exp[i] = robot_joints_filtered.head[i];
    }
    for (int i = 0; i < WAIST_DOF; i++)
    {
        robot_send_info_.joint_q_waist[i] = robot_current_joints.waist[i];
        robot_send_info_.joint_q_waist_exp[i] = robot_joints_filtered.waist[i];
    }
    for (int i = 0; i < ARM_DOF; i++)
    {
        robot_send_info_.joint_q_arm[0][i] = robot_current_joints.left_arm[i];
        robot_send_info_.joint_q_arm[1][i] = robot_current_joints.right_arm[i];
        robot_send_info_.joint_q_arm_exp[0][i] = robot_joints_filtered.left_arm[i];
        robot_send_info_.joint_q_arm_exp[1][i] = robot_joints_filtered.right_arm[i];
        robot_send_info_.arm_cartesion[0][i] = robot_current_pos.left_arm[i];
        robot_send_info_.arm_cartesion[1][i] = robot_current_pos.right_arm[i];
    }
}

void Motion::robotMoveJoint(RobotData::JointCmd &joint_cmd_)
{
    for (int i = 0; i < 2; i++)
    {
        robot_move_joints.head[i] = joint_cmd_.basic_cmd_info.q_exp_head[i];
    }
    for (int i = 0; i < 3; i++)
    {
        robot_move_joints.waist[i] = joint_cmd_.basic_cmd_info.q_exp_waist[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_move_joints.left_arm[i] = joint_cmd_.basic_cmd_info.ee_motion[0][i];
        robot_move_joints.right_arm[i] = joint_cmd_.basic_cmd_info.ee_motion[1][i];
    }
    setFilterJoints(robot_move_joints);
}

void Motion::robotMoveCartesion(RobotData::JointCmd &joint_cmd_)
{
    /**
     * @brief 根据关节指令移动机器人到笛卡尔坐标系下的指定位置
     *
     * 本函数将机器人的关节指令转换为笛卡尔坐标系下的运动指令，并进行滤波处理
     * 目前的实现中，头部和腰部的运动仍然使用关节角而非笛卡尔坐标系
     *
     * @param joint_cmd_ 机器人关节指令，包含基本命令信息和期望的关节角度
     */
    for (int i = 0; i < 2; i++)
    {
        // TODO 头未使用坐标系，而是关节角
        // robot_move_cartesion.head[i] = joint_cmd_.basic_cmd_info.q_exp_head[i];
        robot_move_cartesion.head[i] = robot_current_joints.head[i];
    }
    for (int i = 0; i < 3; i++)
    {
        // TODO 头未使用坐标系，而是关节角
        // robot_move_cartesion.waist[i] = joint_cmd_.basic_cmd_info.q_exp_waist[i];
        robot_move_cartesion.waist[i] = robot_current_joints.waist[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_move_cartesion.left_arm[i] = joint_cmd_.basic_cmd_info.arm_cartesion[0][i];
        robot_move_cartesion.right_arm[i] = joint_cmd_.basic_cmd_info.arm_cartesion[1][i];
    }
    
    // 线性插补robot_move_cartesion[x y z r p y beta],增添函数
    
    calnum_Interp_L(robot_current_pos, robot_move_cartesion, num_interp); 
    DriverBase::RobotJoints robot_move_cartesion_interp[num_interp.max];
    cartesionPlan_L(robot_current_pos, robot_move_cartesion, robot_move_cartesion_interp, num_interp);
    for (int i =0; i<num_interp.max; i++)
    {
        cartesion2Joints(robot_move_cartesion_interp[i], robot_move_joints);
        setFilterJoints(robot_move_joints);
    }

    // cartesion2Joints(robot_move_cartesion_interp, robot_move_joints);
    // setFilterJoints(robot_move_joints);
} /*  */

void Motion::calnum_Interp_L(DriverBase::RobotJoints &cur, DriverBase::RobotJoints &tar, NumPlan &num)
{
    using Vector3D = std::vector<double>;
    // using Vector6D = std::vector<double>;
    Vector3D cur_pos_left = {cur.left_arm[0], cur.left_arm[1], cur.left_arm[2]};
    Vector3D tar_pos_left = {tar.left_arm[0], tar.left_arm[1], tar.left_arm[2]};
    Vector3D cur_pos_right = {cur.right_arm[0], cur.right_arm[1], cur.right_arm[2]};
    Vector3D tar_pos_right = {tar.right_arm[0], tar.right_arm[1], tar.right_arm[2]};
    Vector3D vec3_left = {tar_pos_left[0] - cur_pos_left[0], tar_pos_left[1] - cur_pos_left[1], tar_pos_left[2] - cur_pos_left[2]};
    Vector3D vec3_right = {tar_pos_right[0] - cur_pos_right[0], tar_pos_right[1] - cur_pos_right[1], tar_pos_right[2] - cur_pos_right[2]};
    double totoal_dis_left = sqrt(pow((vec3_left[0]), 2) + pow((vec3_left[1]), 2) + pow((vec3_left[2]), 2));
    double totoal_dis_right = sqrt(pow((vec3_right[0]), 2) + pow((vec3_right[1]), 2) + pow((vec3_right[2]), 2));
    num.left_arm = static_cast<int>(std::ceil(totoal_dis_left / DIS_INTERP))+1;
    num.right_arm = static_cast<int>(std::ceil(totoal_dis_right / DIS_INTERP))+1;
    // Vector6D vec6_left = vec3_left;
    // vec6_left.insert(vec6_left.end(), {tar_pos_left[3] - cur_pos_left[3], tar_pos_left[4] - cur_pos_left[4], tar_pos_left[5] - cur_pos_left[5]});
    // Vector6D vec6_right = vec3_right;
    // vec6_right.insert(vec6_right.end(), {tar_pos_right[3] - cur_pos_right[3], tar_pos_right[4] - cur_pos_right[4], tar_pos_right[5] - cur_pos_right[5]});

    if (num.left_arm > num.right_arm){
        num.max = num.left_arm;
        num.flag = 0;
    } else{
        num.max = num.right_arm;
        num.flag = 1;
    }
    // num.step_vec6_left = divide(vec6_left, num.left_arm-1);
    // num.step_vec6_right = divide(vec6_right, num.right_arm-1);
}


void Motion::cartesionPlan_L(DriverBase::RobotJoints &cur, RobotJoints &tar, RobotJoints &interp, NumPlan &num)
{
    // 计算当前位置和目标位置的间隔，虽然有提前定义，但是插值点数要取整，因此实际插值间隔可能会偏离定义插值间隔，需重新计算校准
    double interval_left[6];
    double interval_right[6];
    for (int i = 0; i < 6; i++)
    {
        interval_left[i] = (tar.left_arm[i] - cur.left_arm[i])/(num.left_arm - 1);
        interval_right[i] = (tar.right_arm[i] - cur.right_arm[i])/(num.right_arm - 1);
    }
    // 生成等距点
    for (int i = 0; i < num.left_arm; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            interp[i].left_arm[j] = cur.left_arm[j] + interval[j]*i;
        }
    }
    for (int i = 0; i < num.right_arm; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            interp[i].right_arm[j] = cur.right_arm[j] + interval[j]*i;
        }
    }
    // 左右臂数据对齐
    int delta;
    delta = (num.max - num.left_arm) + (num.max - num.right_arm);
    if (num.flag == 0)
    {
        for (int i = num.right_arm; i < num.max; i++)
        {
            interp[i].right_arm = interp[num.right_arm].right_arm;
        }
    }else{
        for (int i = num.left_arm; i < num.max; i++)
        {
            interp[i].left_arm = interp[num.left_arm].left_arm;
        }
    }
}


void Motion::cartesion2Joints(DriverBase::RobotJoints &cartesion, DriverBase::RobotJoints &joints)
{
    /**
     * @brief 机器人逆解--数值解
     * 将机器人左右臂关节角转化为为位姿
     * 转换后的结果放入到robot_move_joints中
     */
    double left_pos[ARM_DOF], right_pos[ARM_DOF],
        left_ik_joints[ARM_DOF], right_ik_joints[ARM_DOF];
    int left_ik_state, right_ik_state;
    memcpy(joints.head, cartesion.head, sizeof(float) * HEAD_DOF);
    memcpy(joints.waist, cartesion.waist, sizeof(float)* WAIST_DOF);   
    for (int i = 0; i < ARM_DOF; i++)
    {
        left_pos[i] = static_cast<double>(cartesion.left_arm[i]);
        right_pos[i] = static_cast<double>(cartesion.right_arm[i]);
    }
    _ik(left_pos, 0, left_ik_joints);
    _ik(right_pos, 1, right_ik_joints);
    for (int i = 0; i < ARM_DOF; i++)
    {
        joints.left_arm[i] = static_cast<float>(driver->radToDeg(left_ik_joints[i]));
        joints.right_arm[i] = static_cast<float>(driver->radToDeg(right_ik_joints[i]));
    }
    
}

void Motion::joints2Cartesion(DriverBase::RobotJoints &joints, DriverBase::RobotJoints &cartesion)
{
    double left_joints[ARM_DOF], right_joints[ARM_DOF],
        left_cart[ARM_DOF], right_cart[ARM_DOF];
    for (int i = 0; i < ARM_DOF; i++)
    {
        left_joints[i] = static_cast<double>(driver->degToRad(joints.left_arm[i]));
        right_joints[i] = static_cast<double>(driver->degToRad(joints.right_arm[i]));
    }
    _fk(left_joints, 0, left_cart);
    _fk(right_joints, 1, right_cart);
    for (int i = 0; i < ARM_DOF; i++)
    {
        cartesion.left_arm[i] = static_cast<float>(left_cart[i]);
        cartesion.right_arm[i] = static_cast<float>(right_cart[i]);
    }
}

void Motion::_ik(double *pos, bool l_or_r, double *ik_joint)
{
    double cur_joint[ARM_DOF], limit[ARM_DOF * 2], upper_limit[ARM_DOF];
    int ik_state = 0;
    if (l_or_r == 0)
    {
        for (int i = 0; i < ARM_DOF; i++)
        {
            cur_joint[i] = static_cast<double>(driver->degToRad(robot_current_joints.left_arm[i]));
        }
        memcpy(limit, motion_config.left_arm.joints_limit, sizeof(double) * ARM_DOF * 2);
    }
    else
    {
        for (int i = 0; i < ARM_DOF; i++)
        {
            cur_joint[i] = static_cast<double>(driver->degToRad(robot_current_joints.right_arm[i]));
        }
        memcpy(limit, motion_config.right_arm.joints_limit, sizeof(double) * ARM_DOF * 2);
    }
    ik_7dof_ofst(pos[3], pos[4], pos[5],
                 pos[0], pos[1], pos[2], pos[6],
                 cur_joint, l_or_r, LOrR, FOrB,
                 limit,
                 ik_joint, &ik_state);
    // TODO 判断ik结果
    if (ik_state != 0)
    {
        std::cout << "[Motion]: Ik error, l_or_r: " << l_or_r << " ik state:" << ik_state << std::endl;
        for (int i = 0; i < ARM_DOF; i++)
        {
            cur_joint[i] = static_cast<double>(robot_current_joints.left_arm[i]);
        }
    }
}

void Motion::_fk(double *fk_joints, bool l_or_r, double *cart)
{
    /**
     * @brief 逆解--数值解
     * @param fk_joints 当前关节角的弧度
     * @param l_or_r 0：左臂 1：右臂
     * @param cart 笛卡尔坐标 静态变化zyx
     */
    // std::cout << "[Motion]: joints: " << l_or_r << std::endl;
    // for (int i = 0; i < ARM_DOF; i++)
    // {
    //     std::cout << fk_joints[i] << " ";
    // }
    // std::cout << std::endl;
    int LOrR, FOrB;
    double carte[3], eulVal[3];
    double bet;
    if (l_or_r == 0)
    {
        forward_kinematic_with_ofst(fk_joints, motion_config.left_arm.dh.a_arr,
                                    motion_config.left_arm.dh.alpha_arr,
                                    motion_config.left_arm.dh.d_arr,
                                    motion_config.left_arm.dh.theta_arr,
                                    carte, eulVal, &bet,
                                    &LOrR, &FOrB);
    }
    else
    {
        forward_kinematic_with_ofst(fk_joints, motion_config.right_arm.dh.a_arr,
                                    motion_config.right_arm.dh.alpha_arr,
                                    motion_config.right_arm.dh.d_arr,
                                    motion_config.right_arm.dh.theta_arr, carte,
                                    eulVal, &bet, &LOrR,
                                    &FOrB);
    }
    // std::cout << "[Motion]: a_arr: ";
    // for (int i = 0; i < ARM_DOF; i++)
    // {
    //     std::cout << motion_config.right_arm.dh.a_arr[i] << " ";
    // }
    // std::cout << "[Motion]: alpha_arr: ";
    // for (int i = 0; i < ARM_DOF; i++)
    // {
    //     std::cout << motion_config.right_arm.dh.alpha_arr[i] << " ";
    // }
    // std::cout << "[Motion]: d_arr: ";
    // for (int i = 0; i < ARM_DOF; i++)
    // {
    //     std::cout << motion_config.right_arm.dh.d_arr[i] << " ";
    // }
    // std::cout << "[Motion]: theta_arr: ";
    // for (int i = 0; i < ARM_DOF; i++)
    // {
    //     std::cout << motion_config.right_arm.dh.theta_arr[i] << " ";
    // }
    cart[0] = carte[0];
    cart[1] = carte[1];
    cart[2] = carte[2];
    cart[5] = eulVal[0];
    cart[4] = eulVal[1];
    cart[3] = eulVal[2];
    cart[6] = bet;
    // std::cout << std::endl;
    // for (int i = 0; i < ARM_DOF; i++)
    // {
    //     std::cout << cart[i] << " ";
    // }
    // std::cout << std::endl;
}

void Motion::resetMotionError()
{
    driver->reset_driver_error();
}

void Motion::loadMotionConfig(std::string path)
{
    /**
     * 从YAML文件加载运动配置
     *
     * @param path YAML文件的路径
     *
     * 此函数读取指定路径下的YAML文件，从中提取机器人的左臂和右臂的运动配置数据
     * 配置数据包括DH参数和关节限制，这些参数对于机器人的运动学计算至关重要
     * 如果YAML文件解析失败，函数将打印错误信息
     */
    YAML::Node config = YAML::LoadFile(path);
    if (!config)
    {
        std::cerr << "Failed to parse YAML file" << std::endl;
    }
    const YAML::Node &left_arm_node = config["robot"]["left_arm"];
    for (size_t i = 0; i < ARM_DOF; ++i)
    {
        motion_config.left_arm.dh.a_arr[i] = left_arm_node["dh"]["a_arr"][i].as<double>();
        motion_config.left_arm.dh.alpha_arr[i] = left_arm_node["dh"]["alpha_arr"][i].as<double>();
        motion_config.left_arm.dh.d_arr[i] = left_arm_node["dh"]["d_arr"][i].as<double>();
        motion_config.left_arm.dh.theta_arr[i] = left_arm_node["dh"]["theta_arr"][i].as<double>();
        motion_config.left_arm.joints_limit[i * 2] = left_arm_node["joints_limit"]["lower_limit"][i].as<double>();
        motion_config.left_arm.joints_limit[i * 2 + 1] = left_arm_node["joints_limit"]["upper_limit"][i].as<double>();
    }
    // 填充右臂数据（类似地）
    const YAML::Node &right_arm_node = config["robot"]["right_arm"];
    for (size_t i = 0; i < ARM_DOF; ++i)
    {
        motion_config.right_arm.dh.a_arr[i] = right_arm_node["dh"]["a_arr"][i].as<double>();
        motion_config.right_arm.dh.alpha_arr[i] = right_arm_node["dh"]["alpha_arr"][i].as<double>();
        motion_config.right_arm.dh.d_arr[i] = right_arm_node["dh"]["d_arr"][i].as<double>();
        motion_config.right_arm.dh.theta_arr[i] = right_arm_node["dh"]["theta_arr"][i].as<double>();
        motion_config.right_arm.joints_limit[i * 2] = right_arm_node["joints_limit"]["lower_limit"][i].as<double>();
        motion_config.right_arm.joints_limit[i * 2 + 1] = right_arm_node["joints_limit"]["upper_limit"][i].as<double>();
    }
}

Motion::~Motion()
{
}
