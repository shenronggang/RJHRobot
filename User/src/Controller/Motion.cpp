#include "Motion.hpp"

// TODO 将driver放入Motion中维护
Motion::Motion(const std::string &driver_name) : driver_name(driver_name), filter_enable(true)
{
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
    driver->get_robot_joints(robot_current_joints);
    initFilterJoints(robot_current_joints);
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
        if (motion_state == MotionState::RUN && motor_on == 0)
        {
            if (filter_enable)
            {
                // getFilterJoints(robot_joints_filtered);
                driver->set_robot_joints(robot_joints_filtered);
                // std::cout << "robot move joints by filter " << robot_joints_filtered.left_arm[4] << std::endl;
            }
        }
        // else if (motion_state == MotionState::READY_OK && motor_on == 0)
        // {
        //     driver->get_robot_joints(robot_current_joints);
        //     driver->set_robot_joints(robot_current_joints);
        //     std::cout << "robot move joints by current " << std::endl;
        // }
        else
        {
            // std::cout << "robot move joints by current " << std::endl;
        }
        usleep(1000);
    }
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
void Motion::initFilterJoints(DriverBase::RobotJoints &robot_joint)
{
    for (int i = 0; i < 2; i++)
    {
        head_filters[i].init(robot_joint.head[i]);
    }
    for (int i = 0; i < 3; i++)
    {
        waist_filters[i].init(robot_joint.waist[i]);
    }
    for (int i = 0; i < 7; i++)
    {
        left_arm_filters[i].init(robot_joint.left_arm[i]);
        right_arm_filters[i].init(robot_joint.right_arm[i]);
    }
}

void Motion::enableRobot()
{
    motor_on = driver->enable_robot();
}

void Motion::diableRobot()
{
    driver->disable_robot();
}

void Motion::getCurrentJoints(RobotData::RobotInfo &robot_info_)
{
    driver->get_robot_joints(robot_current_joints);
    for (int i = 0; i < 2; i++)
    {
        robot_info_.robot_send_info_.joint_q_head[i] = robot_current_joints.head[i];
        robot_info_.robot_send_info_.joint_q_head_exp[i] = robot_joints_filtered.head[i];
    }
    for (int i = 0; i < 3; i++)
    {
        robot_info_.robot_send_info_.joint_q_waist[i] = robot_current_joints.waist[i];
        robot_info_.robot_send_info_.joint_q_waist_exp[i] = robot_joints_filtered.waist[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_info_.robot_send_info_.joint_q_arm[0][i] = robot_current_joints.left_arm[i];
        robot_info_.robot_send_info_.joint_q_arm[1][i] = robot_current_joints.right_arm[i];
        robot_info_.robot_send_info_.joint_q_arm_exp[0][i] = robot_joints_filtered.left_arm[i];
        robot_info_.robot_send_info_.joint_q_arm_exp[1][i] = robot_joints_filtered.right_arm[i];
    }
}

void Motion::robotMoveJoint(RobotData::RobotInfo &robot_info_)
{
    for (int i = 0; i < 2; i++)
    {
        robot_move_joints.head[i] = robot_info_.joint_cmd_.basic_cmd_info.q_exp_head[i];
    }
    for (int i = 0; i < 3; i++)
    {
        robot_move_joints.waist[i] = robot_info_.joint_cmd_.basic_cmd_info.q_exp_waist[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_move_joints.left_arm[i] = robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][i];
        robot_move_joints.right_arm[i] = robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][i];
    }
    setFilterJoints(robot_move_joints);
}

void Motion::robotMoveCartesion(RobotData::RobotInfo &robot_info_)
{
    for (int i = 0; i < 2; i++)
    {
        robot_move_joints.head[i] = robot_info_.joint_cmd_.basic_cmd_info.q_exp_head[i];
    }
    for (int i = 0; i < 3; i++)
    {
        robot_move_joints.waist[i] = robot_info_.joint_cmd_.basic_cmd_info.q_exp_waist[i];
    }
    for (int i = 0; i < 7; i++)
    {
        robot_move_joints.left_arm[i] = robot_info_.joint_cmd_.basic_cmd_info.ee_motion[0][i];
        robot_move_joints.right_arm[i] = robot_info_.joint_cmd_.basic_cmd_info.ee_motion[1][i];
    }
    double z = 56.3387;
    for (int i = 0; i < 20; i++)
    {
        
        driver->get_robot_joints(robot_current_joints);
        robot_move_joints = robot_current_joints;
        double cur_theta[7];
        for (int i = 0; i < 7; ++i)
        {
            cur_theta[i] = static_cast<double>(robot_current_joints.left_arm[i]);
        }
        // TODO: get current cartesion;
        // left arm
        double x = -338.5171; //(mm)
        double y = 328.7387;

        double z_alpha = 1.3135; //% rad
        double y_beta = 1.3982;
        double x_gamma = -2.8824;
        double bet = 0.2387;
        const double *const_cur_theta = cur_theta;
        int b_lt_or_rt = 1;
        int FOrB = 1;
        int LOrR = 1;
        double theta[7];
        int ik_state;
        // b_lt_or_rt-- 暂时还没有用到
        // cur_theta-- 上一时刻关节角的值
        ik_7dof_ofst(z_alpha, y_beta, x_gamma, x, y, z, bet,
                     const_cur_theta, b_lt_or_rt, LOrR, FOrB,
                     theta, &ik_state);
        if (ik_state == 0)
        {
            for (int i = 0; i < 7; i++)
            {
                robot_move_joints.left_arm[i] = static_cast<float>(driver->radToDeg(theta[i]));
            }
        }
        driver->set_robot_joints(robot_move_joints);
        setFilterJoints(robot_move_joints);
        sleep(0.5);
        z += 2;
    }
}

Motion::~Motion()
{
}
