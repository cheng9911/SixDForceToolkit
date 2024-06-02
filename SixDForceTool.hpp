
// * @brief SixDForceTool类功能函数：
// *1. 负载参数辩识（质量、重心）   六点标定，六个姿态，六个六维力的平均值
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

    SixDForce(double fx = 0.0, double fy = 0.0, double fz = 0.0, double tr = 0.0, double tp = 0.0, double ty = 0.0)
        : force_x(fx), force_y(fy), force_z(fz), torque_roll(tr), torque_pitch(tp), torque_yaw(ty) {}
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

struct ZeroOffset
{
    double zero_offset_force_x;
    double zero_offset_force_y;
    double zero_offset_force_z;
    double zero_offset_torque_roll;
    double zero_offset_torque_pitch;
    double zero_offset_torque_yaw;

    ZeroOffset(double zero_offset_force_x = 0.0, double zero_offset_force_y = 0.0, double zero_offset_force_z = 0.0,
                 double zero_offset_torque_roll = 0.0, double zero_offset_torque_pitch = 0.0, double zero_offset_torque_yaw = 0.0)
                 : zero_offset_force_x(zero_offset_force_x), zero_offset_force_y(zero_offset_force_y), zero_offset_force_z(zero_offset_force_z),
                 zero_offset_torque_roll(zero_offset_torque_roll), zero_offset_torque_pitch(zero_offset_torque_pitch), zero_offset_torque_yaw(zero_offset_torque_yaw) {}
    // 注意这里提供了默认参数，使得构造函数可以被无参数调用
};


class SixDForceTool
{
private:
    /* data */
    
    
    double m_gravity = 9.8;
    double m_zero_offset_force_x = 0.0;
    double m_zero_offset_force_y = 0.0;
    double m_zero_offset_force_z = 0.0;
    double m_zero_offset_torque_roll = 0.0;
    double m_zero_offset_torque_pitch = 0.0;
    double m_zero_offset_torque_yaw = 0.0;
    

public:
    SixDForceTool(double mass = 0.0, double massx = 0.0, double massy = 0.0, double massz = 0.0)
        : m_mass(mass), m_massx(massx), m_massy(massy), m_massz(massz) {
        // 清除poses 和 forces
        poses.clear();
        forces.clear();
        forces.resize(6);

        }
    ~SixDForceTool();
    double m_mass;
    double m_massx;
    double m_massy;
    double m_massz;
    std::vector<Pose> poses;
    std::vector<SixDForce> forces;
    KDL::Wrench wrench_used;


    void addData(const Pose& pose, const SixDForce& force);
    void addData(const SixDForce& force);
    // * @brief 负载参数辩识（质量、重心）
    // * @input param
    // * @output
    int LoadParameterIdentification(const KDL::Wrench &wrench, const KDL::Rotation &R);
    KDL::Wrench GravityCompensation(const KDL::Wrench &wrench, const KDL::Rotation &R);
    void SensorDataFilter();
    // * @brief 获取质量和重心
    // * @input param
    // * @output MassResult 质量、重心x、重心y、重心z
    MassResult GetMassAndGravity();
    
    // * @brief 获取零偏
    // * @input param
    // * @output ZeroOffset 零偏
    ZeroOffset GetZeroOffset();
    
};

SixDForceTool::~SixDForceTool()
{
}
void SixDForceTool::addData(const Pose& pose, const SixDForce& force)
{
    poses.push_back(pose);
    forces.push_back(force);
}

void SixDForceTool::addData(const SixDForce& force)
{   
    forces.push_back(force);
}

int SixDForceTool::LoadParameterIdentification(const KDL::Wrench &wrench, const KDL::Rotation &R)
{
    // 1. 负载参数辩识（质量、重心）
    // 六点标定，六个姿态，六个六维力的平均值
    // 2. 重力补偿
    // 3. 传感器数据滤波
    std::cout << "LoadParameterIdentification" << std::endl;
    if(forces.size() != 6)
    {
        std::cout << "forces.size() != 6,无法进行参数辩识" << std::endl;
        return -1;
    }
    // 负载参数辩识公式
    MassResult massResult = GetMassAndGravity();
   
    ZeroOffset zeroOffset = GetZeroOffset();
    

    // 重力补偿公式
    KDL::Wrench wrench_Compensation = GravityCompensation(wrench, R);
    

    // 传感器数据滤波公式


    wrench_used = wrench_Compensation;
    return 0;
}


// 函数实现放在这里
/**
 * 根据硕士论文《装配机器人作业过程控制系统应用与软件开发》进行重力补偿
 * @param forces : 根据论文方法测量的6个姿态下力传感器数据
 */
MassResult SixDForceTool::GetMassAndGravity()
{
    m_mass = (forces[1].force_z - forces[0].force_z + forces[3].force_x -forces[2].force_x + forces[5].force_y - forces[4].force_y)/6.0;
    m_massx = (forces[0].torque_pitch - forces[1].torque_pitch + forces[5].torque_yaw - forces[4].torque_yaw)/(4.0*m_mass);
    m_massy = (forces[1].torque_roll - forces[0].torque_roll + forces[2].torque_yaw - forces[3].torque_yaw)/(4.0*m_mass);
    m_massz = (forces[3].torque_pitch - forces[2].torque_pitch + forces[4].torque_roll - forces[5].torque_roll)/(4.0*m_mass);
   
    return MassResult(m_mass, m_massx, m_massy, m_massz);
}

ZeroOffset SixDForceTool::GetZeroOffset()
{
    // 计算零偏
    m_zero_offset_force_x = (forces[2].force_x + forces[3].force_x)/2.0;
    m_zero_offset_force_y = (forces[4].force_y + forces[5].force_y)/2.0;
    m_zero_offset_force_z = (forces[0].force_z + forces[1].force_z)/2.0;
    m_zero_offset_torque_roll = (forces[0].torque_roll + forces[1].torque_roll + forces[4].torque_roll + forces[5].torque_roll)/4.0;
    m_zero_offset_torque_pitch = (forces[0].torque_pitch + forces[1].torque_pitch + forces[2].torque_pitch + forces[3].torque_pitch)/4.0;
    m_zero_offset_torque_yaw = (forces[2].torque_yaw + forces[3].torque_yaw +forces[4].torque_yaw + forces[5].torque_yaw)/4.0;
    // std::cout<<"m_zero_offset_force_x:"<<m_zero_offset_force_x<<std::endl;
    // std::cout<<"m_zero_offset_force_y:"<<m_zero_offset_force_y<<std::endl;
    // std::cout<<"m_zero_offset_force_z:"<<m_zero_offset_force_z<<std::endl;

    return ZeroOffset(m_zero_offset_force_x, m_zero_offset_force_y, m_zero_offset_force_z, m_zero_offset_torque_roll, m_zero_offset_torque_pitch, m_zero_offset_torque_yaw);
}


/**
 * 根据力传感器测量的力，和力传感器姿态对重力进行补偿
 * @param wrench： 力传感器测得的力
 * @param R     ： 力传感器姿态（注意这里是力传感器姿态不是末端姿态）
 * @return      ： 返回补偿后的力
 */
KDL::Wrench SixDForceTool::GravityCompensation(const KDL::Wrench &wrench, const KDL::Rotation &R)
{
    // 计算重力补偿
    KDL::Wrench compensated_wrench ;
    //KDL::Vector G_v = -m_mass * KDL::Vector(R.data[6], R.data[7], R.data[8]);
    KDL::Vector G_v = R.Inverse() * KDL::Vector(0,0,-m_mass);

    // std::cout<<"G_v:"<<G_v[0]<<std::endl;
    // std::cout<<"G_v:"<<G_v[1]<<std::endl;
    // std::cout<<"G_v:"<<G_v[2]<<std::endl;

    compensated_wrench.force.data[0] = wrench.force.data[0] - G_v[0] - m_zero_offset_force_x;
    compensated_wrench.force.data[1] = wrench.force.data[1] - G_v[1] - m_zero_offset_force_y;
    compensated_wrench.force.data[2] = wrench.force.data[2] - G_v[2] - m_zero_offset_force_z;

    compensated_wrench.torque.data[0] = wrench.torque.data[0] - G_v[2] * m_massy + G_v[1] * m_massz - m_zero_offset_torque_roll;
    compensated_wrench.torque.data[1] = wrench.torque.data[1] - G_v[0] * m_massz + G_v[2] * m_massx - m_zero_offset_torque_pitch;
    compensated_wrench.torque.data[2] = wrench.torque.data[2] - G_v[1] * m_massx + G_v[0] * m_massy - m_zero_offset_torque_yaw;

    wrench_used = compensated_wrench;
    return compensated_wrench;
}

#endif // SIXDFORCETOOL_HPP