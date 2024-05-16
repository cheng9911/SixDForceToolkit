
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

// ! 定义姿态结构体，默认是RPY角，是六维力相对于基座坐标系的姿态，即六维力的旋转角度,不是法兰盘的旋转角度
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
    KDL::Wrench GetGravityCompensation(KDL::Rotation R, KDL::Wrench wrench_origin);
};

SixDForceTool::~SixDForceTool()
{
}
void SixDForceTool::addData(const Pose &pose, const SixDForce &force)
{
    poses.push_back(pose);
    forces.push_back(force);
}

// 四点法负载参数辩识
int SixDForceTool::LoadParameterIdentification(int n)
{
    // 1. 负载参数辩识（质量、重心）
    // 四点标定，四个姿态，四个六维力的平均值

    if (poses.size() != n || forces.size() != n)
    {
        std::cout << "poses.size() !=  || forces.size() != , 存储点位不足" << std::endl;
        return -1;
    }
    // if (n < 4)
    // {
    //     std::cout << "n<4, 采样点数不足" << std::endl;
    //     return -1;
    // }

    /*
     *step1: 求解负载质心
     *step2: 求解负载质量，零点，世界坐标系和基座坐标系的偏移角度
     *公式参见https://qd8bd52pvj.feishu.cn/wiki/wikcna2rL08ohqnC3HCqHeK1fjg?from=from_copylink
     */
    // step1 六维力产生的力和力矩通过addData函数添加到poses和forces中
    // 求解负载质心

    Eigen::MatrixXd F(3 * n, 6);
    Eigen::MatrixXd M(3 * n, 1);
    Eigen::MatrixXd F_temp(3, 6);
    Eigen::MatrixXd M_temp(3, 1);
    Eigen::MatrixXd A(6, 1);
    double error;
    // 求解负载质量，零点，世界坐标系和基座坐标系的偏移角度
    KDL::Rotation R_temp; // 存放六维力相对于基座坐标系的旋转矩阵
    for (int i = 0; i < n; i++)
    {
        F_temp << 0, forces[i].force_z, -forces[i].force_y, 1, 0, 0,
            -forces[i].force_z, 0, forces[i].force_x, 0, 1, 0,
            forces[i].force_y, -forces[i].force_x, 0, 0, 0, 1;
        M_temp << forces[i].torque_roll, forces[i].torque_pitch, forces[i].torque_yaw;
        F.block(3 * i, 0, 3, 6) = F_temp;
        M.block(3 * i, 0, 3, 1) = M_temp;
    }
    // 最小二乘法求解负载质心，（F^T*F)^-1*F^T*M
    // A=[massx,massy,massz,k1,k2,k3]
    A = (F.transpose() * F).inverse() * F.transpose() * M;
    error = (F * A - M).norm();
    // 输出最小二乘法求解的误差
    std::cout << "质心的误差: " << error << std::endl;
    m_massx = A(0, 0);
    m_massy = A(1, 0);
    m_massz = A(2, 0);
    double k1 = A(3, 0);
    double k2 = A(4, 0);
    double k3 = A(5, 0);
    // std::cout << "m_massx:" << m_massx << std::endl;
    // std::cout << "m_massy:" << m_massy << std::endl;
    // std::cout << "m_massz:" << m_massz << std::endl;
    // std::cout << "K1" << k1 << std::endl;
    // std::cout << "K2" << k2 << std::endl;
    // std::cout << "K3" << k3 << std::endl;

    // step2 求解负载质量，零点，世界坐标系和基座坐标系的偏移角度
    // 清空F和M
    F.setZero();
    M.setZero();
    A.setZero();
    // 重新填充F和M
    for (int i = 0; i < n; i++)
    {
        R_temp = KDL::Rotation::RPY(poses[i].roll, poses[i].pitch, poses[i].yaw);
        R_temp = R_temp.Inverse();

        F_temp << R_temp(0, 0), R_temp(0, 1), R_temp(0, 2), 1, 0, 0,
            R_temp(1, 0), R_temp(1, 1), R_temp(1, 2), 0, 1, 0,
            R_temp(2, 0), R_temp(2, 1), R_temp(2, 2), 0, 0, 1;
        M_temp << forces[i].force_x, forces[i].force_y, forces[i].force_z;
        F.block(3 * i, 0, 3, 6) = F_temp;
        M.block(3 * i, 0, 3, 1) = M_temp;
    }
    // 最小二乘法求解负载质量，（F^T*F)^-1*F^T*M
    // A=[Lx,Ly,Lz,ZeroForceX,ZeroForceY,ZeroForceZ]

    A = (F.transpose() * F).inverse() * F.transpose() * M;

    // SVD
    // Eigen::MatrixXd x = F.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(M);
    // std::cout<< x << std::endl;
    // A= x;
    // 最小二乘法求解的误差
   
    error = (F * A - M).norm();
    // 输出最小二乘法求解的误差
    std::cout << "质量，零漂的误差: " << error << std::endl;
    double det = (F.transpose() * F).determinant();

    // 输出矩阵的行列式
    std::cout << "The determinant of the matrix is: " << det << std::endl;
    
    double G = sqrt(A(0, 0) * A(0, 0) + A(1, 0) * A(1, 0) + A(2, 0) * A(2, 0));

    m_mass = G / m_gravity;
    U = asin(-A(1, 0) / G);
    V = atan(-A(0, 0) / A(2, 0));
    ZeroForceX = A(3, 0);
    ZeroForceY = A(4, 0);
    ZeroForceZ = A(5, 0);
    ZeroTorqueRoll = k1 - ZeroForceY * m_massz + ZeroForceZ * m_massy;
    ZeroTorquePitch = k2 - ZeroForceZ * m_massx + ZeroForceX * m_massz;
    ZeroTorqueYaw = k3 - ZeroForceX * m_massy + ZeroForceY * m_massx;
    std::cout << "G:" << G << std::endl;
    std::cout << "U:" << U * 180 / M_PI << std::endl;
    std::cout << "V:" << V * 180 / M_PI << std::endl;
    std::cout << "ZeroForceX:" << ZeroForceX << std::endl;
    std::cout << "ZeroForceY:" << ZeroForceY << std::endl;
    std::cout << "ZeroForceZ:" << ZeroForceZ << std::endl;

    return 0;
}

// 函数实现放在这里
MassResult SixDForceTool::GetMassAndGravity()
{

    return MassResult(m_mass, m_massx, m_massy, m_massz);
}

SixDForce SixDForceTool::GetZeroDriftCalibration()
{
    return SixDForce(ZeroForceX, ZeroForceY, ZeroForceZ, ZeroTorqueRoll, ZeroTorquePitch, ZeroTorqueYaw);
}
WorldBaseOffset SixDForceTool::GetWorldBaseOffset()
{
    return WorldBaseOffset(U, V);
}
KDL::Wrench SixDForceTool::GetGravityCompensation(KDL::Rotation R, KDL::Wrench wrench_origin)
{
    KDL::Wrench wrench_compensation;

    KDL::Vector force_G = R.Inverse() * KDL::Vector(0, -0, m_mass * m_gravity);
    KDL::Rotation cross_mass = KDL::Rotation(0, -m_massz, m_massy,
                                             m_massz, 0, -m_massx,
                                             -m_massy, m_massx, 0);

    wrench_compensation.force = wrench_origin.force - force_G - KDL::Vector(ZeroForceX, ZeroForceY, ZeroForceZ);
    wrench_compensation.torque = wrench_origin.torque - cross_mass * wrench_compensation.force - KDL::Vector(ZeroTorqueRoll, ZeroTorquePitch, ZeroTorqueYaw);

    return wrench_compensation;
}

#endif // SIXDFORCETOOL_HPP