#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>

class LowFilter
{
private:
    float theta_cur;
    float theta_out;
    float acc_lim = 16.67; // 单位，rad/s2
    float vel_lim = 2.61;  // 单位，rad/s
    float vel = 0;         // 初始速度,vel是全局变量
    float acc = 20;        // 初始加速度，acc是全局变量
    float deltat = 0;      // 控制周期
    // 以上是运动参数
    float m_lim = 1;                  // 惯量最小限幅
    float k = 500;                      // 刚度系数
    float m = m_lim;                  // 惯量初始值，不可为0
    float km = 1;                    // 变惯量 增益
    unsigned char m_change = 1;       // 变惯量开关
    float b = 2 * 1.01 * sqrt(k * m); // 阻尼比取略＞1，防止超调
    // 以上是滤波器阻抗参数
public:
    LowFilter(unsigned char m_change = 1)
        : theta_cur(0), acc_lim(16.67), vel_lim(2.61),
          vel(0), acc(0), deltat(0.001), m_lim(1), k(500), m(m_lim),
          km(1), m_change(m_change), b(2 * 1.1 * sqrt(k * m))
    {
    }

    ~LowFilter() {}
    float init(float _theta_cur)
    {
        theta_cur = _theta_cur;
        theta_out = theta_cur; // 初始角度，theta_out是全局变量
        return theta_out;
    }

    void update(float _theta_cur)
    {
        theta_cur = _theta_cur;
        // std::cout << "theta_cur: " << theta_cur << std::endl;
    }

    float get()
    {
        float delt_theta = theta_out - theta_cur;
        // theta_out是滤波后压给驱动器的，每个周期都在更新
        if (m_change == 1)
        {
            m = abs(km * delt_theta);
            if (m < m_lim)
                m = m_lim;
            b = 2 * 1.01 * sqrt(k * m);
        }

        acc = (-k * delt_theta - b * vel) / m; // 更新加速度
        if (acc > acc_lim)
            acc = acc_lim;
        else if (acc < -acc_lim)
            acc = -acc_lim;

        vel = vel + acc * deltat; // 更新速度
        if (vel > vel_lim)
            vel = vel_lim;
        else if (vel < -vel_lim)
            vel = -vel_lim;
        theta_out = theta_out + vel * deltat + 0.5 * acc * deltat * deltat;
        return theta_out; // 将theta_out压入关节驱动器
    }
};
