
// * @brief SixDForceTool类功能函数：
// *1. 负载参数辩识（质量、重心）   最少 四点标定，四个姿态，四个六维力的平均值
// *2. https://blog.csdn.net/shuimujieming/article/details/137015352?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522171505339516800211589180%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=171505339516800211589180&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-137015352-null-null.142^v100^pc_search_result_base9&utm_term=%E5%85%AD%E7%BB%B4%E5%8A%9B%E4%BC%A0%E6%84%9F%E5%99%A8%E8%A1%A5%E5%81%BF&spm=1018.2226.3001.4187

#ifndef SIXFORCETOOL_H
#define SIXFORCETOOL_H

#include <iostream>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// 世界坐标系和基座坐标系的偏移角度
struct WorldBaseOffset
{
    double U;
    double V;
    WorldBaseOffset(double u = 0.0, double v = 0.0)
        : U(u), V(v) {}
};

// ! 定义姿态结构体，默认是RPY角，是六维力相对于基座坐标系的姿态，即六维力的旋转,不是法兰盘的旋转
struct Pose
{
    double roll;  // 横滚角
    double pitch; // 俯仰角
    double yaw;   // 偏航角
    Pose(double r = 0.0, double p = 0.0, double y = 0.0)
        : roll(r), pitch(p), yaw(y) {}
    // 注意这里提供了默认参数，使得构造函数可以被无参数调用
};

// 定义六维力结构体
struct SixDForce
{
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

class SixDForceTool
{
private:
    /* data */
    double m_mass;
    double m_massx;
    double m_massy;
    double m_massz;
    double m_gravity = 9.8;
    double ZeroForceX = 0.0;
    double ZeroForceY = 0.0;
    double ZeroForceZ = 0.0;
    double ZeroTorqueRoll = 0.0;
    double ZeroTorquePitch = 0.0;
    double ZeroTorqueYaw = 0.0;
    // 世界坐标系和基座坐标系的偏移角度U是绕世界坐标系x轴的旋转角，V是绕基坐标系的y轴的旋转角,默认是0
    double V = 0.0;
    double U = 0.0;

public:
    SixDForceTool(double mass = 0.0, double massx = 0.0, double massy = 0.0, double massz = 0.0)
        : m_mass(mass), m_massx(massx), m_massy(massy), m_massz(massz)
    {
        // 清除poses 和 forces
        poses.clear();
        forces.clear();
    }
    ~SixDForceTool();
    std::vector<Pose> poses;
    std::vector<SixDForce> forces;
    void addData(const Pose &pose, const SixDForce &force);
    // 零漂移校准
    void ZeroDriftCalibration();
    void SetZeroOffset(double zeroForceX, double zeroForceY, double zeroForceZ, double zeroTorqueRoll, double zeroTorquePitch, double zeroTorqueYaw);
    // * @brief 负载参数辩识（质量、重心）
    // * @input param n 采样点数（对应的姿态和六维力的点数）
    // * @output
    int LoadParameterIdentification(int n = 4);
    void SensorDataFilter();
    // * @brief 获取质量和重心
    // * @input param
    // * @output MassResult 质量、重心x、重心y、重心z
    MassResult GetMassAndGravity();
    SixDForce GetZeroDriftCalibration();
    WorldBaseOffset GetWorldBaseOffset();
    KDL::Wrench GetForceGravityCompensation(KDL::Rotation R, KDL::Wrench wrench_origin);
    void SetMass(double mass, double massx, double massy, double massz);
};


#endif // SIXDFORCETOOL_HPP