
// * @brief SixDForceTool类功能函数：
// *1. 负载参数辩识（质量、重心）   四点标定，四个姿态，四个六维力的平均值
// *2. 重力补偿
// *3. 传感器数据滤波
#ifndef SIXDFORCETOOL_HPP
#define SIXDFORCETOOL_HPP

#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <vector>
// 定义姿态结构体
struct Pose {
    double roll;    // 横滚角
    double pitch;   // 俯仰角
    double yaw;     // 偏航角
};

// 定义六维力结构体
struct SixDForce {
    double force_x;
    double force_y;
    double force_z;
    double torque_roll;
    double torque_pitch;
    double torque_yaw;
};
struct MassResult
{
    double mass;
    double massx;
    double massy;
    double massz;

    MassResult(double m = 0.0, double mx = 0.0, double my = 0.0, double mz = 0.0)
        : mass(m), massx(mx), massy(my), massz(mz) {}
    // 注意这里提供了默认参数，使得构造函数可以被无参数调用
};

class SixDForceTool
{
private:
    /* data */
    double m_mass;
    double m_massx;
    double m_massy;
    double m_massz;
    double m_gravity = 9.8;
    

public:
    SixDForceTool(double mass = 0.0, double massx = 0.0, double massy = 0.0, double massz = 0.0)
        : m_mass(mass), m_massx(massx), m_massy(massy), m_massz(massz) {
        // 清除poses 和 forces
        poses.clear();
        forces.clear();

        }
    ~SixDForceTool();
    std::vector<Pose> poses;
    std::vector<SixDForce> forces;


    void addData(const Pose& pose, const SixDForce& force);
    // * @brief 负载参数辩识（质量、重心）
    // * @input param
    // * @output
    int LoadParameterIdentification();
    void GravityCompensation();
    void SensorDataFilter();
    // * @brief 获取质量和重心
    // * @input param
    // * @output MassResult 质量、重心x、重心y、重心z
    MassResult GetMassAndGravity();
};

SixDForceTool::~SixDForceTool()
{
}
void SixDForceTool::addData(const Pose& pose, const SixDForce& force)
{
    poses.push_back(pose);
    forces.push_back(force);
}


int SixDForceTool::LoadParameterIdentification()
{
    // 1. 负载参数辩识（质量、重心）
    // 四点标定，四个姿态，四个六维力的平均值
    // 2. 重力补偿
    // 3. 传感器数据滤波
    std::cout << "LoadParameterIdentification" << std::endl;
    if(poses.size() != 4 || forces.size() != 4)
    {
        std::cout << "poses.size() != 4 || forces.size() != 4,无法进行参数辩识" << std::endl;
        return -1;
    }
    // 负载参数辩识公式
    


}


// 函数实现放在这里
MassResult SixDForceTool::GetMassAndGravity()
{
    return MassResult(m_mass, m_massx, m_massy, m_massz);
}

#endif // SIXDFORCETOOL_HPP