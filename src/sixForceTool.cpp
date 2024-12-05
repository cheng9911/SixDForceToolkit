/**
  ******************************************************************************
  * @file           : sixForceTool.cpp
  * @author         : sun
  * @brief          : None
  * @attention      : None
  * @date           : 11/27/24
  ******************************************************************************
  */
#include "sixForceTool.h"
SixDForceTool::~SixDForceTool()
{
}
void SixDForceTool::addData(const Pose &pose, const SixDForce &force)
{
    poses.push_back(pose);
    forces.push_back(force);
}

void SixDForceTool::ZeroDriftCalibration()
{
}

void SixDForceTool::SetZeroOffset(double zeroForceX, double zeroForceY, double zeroForceZ, double zeroTorqueRoll,
    double zeroTorquePitch, double zeroTorqueYaw)
{
    ZeroForceX=zeroForceX;
    ZeroForceY=zeroForceY;
    ZeroForceZ=zeroForceZ;
    ZeroTorqueRoll=zeroTorqueRoll;
    ZeroTorquePitch=zeroTorquePitch;
    ZeroTorqueYaw=zeroTorqueYaw;
}

void SixDForceTool::SensorDataFilter()
{
}

KDL::Wrench SixDForceTool::GetForceGravityCompensation(KDL::Rotation R, KDL::Wrench wrench_origin)
{
    KDL::Wrench wrench_compensation;

    KDL::Vector force_G = R.Inverse() * KDL::Vector(0, -0, m_mass * -m_gravity);
    KDL::Rotation cross_mass = KDL::Rotation(0, -m_massz, m_massy,
                                             m_massz, 0, -m_massx,
                                             -m_massy, m_massx, 0);
    wrench_compensation.torque = wrench_origin.torque - cross_mass * force_G - KDL::Vector(ZeroTorqueRoll, ZeroTorquePitch, ZeroTorqueYaw);

    wrench_compensation.force = wrench_origin.force - force_G - KDL::Vector(ZeroForceX, ZeroForceY, ZeroForceZ);

    return wrench_compensation;
}

void SixDForceTool::SetMass(double mass, double massx, double massy, double massz)
{
        
        m_mass = mass;
        m_massx = massx;
        m_massy = massy;
        m_massz = massz;
    

}

// 四点法负载参数辩识
int SixDForceTool::LoadParameterIdentification(int n)
{
    // 1. 负载参数辩识（质量、重心）
    // 最小四点标定，四个姿态，四个六维力的平均值

    if (poses.size() < n || forces.size() < n)
    {
        std::cout << "poses.size() !=  || forces.size() != , 存储点位不足" << std::endl;
        return -1;
    }
    /*
     *step1: 求解负载质心
     *step2: 求解负载质量，零点，世界坐标系和基座坐标系的偏移角度
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
    std::cout << "massx: " << m_massx << ", massy: " << m_massy << ", massz: " << m_massz << std::endl;
    std::cout << "k1: " << k1 << ", k2: " << k2 << ", k3: " << k3 << std::endl;

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
    std::cout<<"A:"<<A(0,0)<<","<<A(1,0)<<","<<A(2,0)<<","<<A(3,0)<<","<<A(4,0)<<","<<A(5,0)<<std::endl;
    error = (F * A - M).norm()/6;
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
    std::cout<<"ZeroTorqueRoll:"<<ZeroTorqueRoll<<std::endl;
    std::cout<<"ZeroTorquePitch:"<<ZeroTorquePitch<<std::endl;
    std::cout<<"ZeroTorqueYaw:"<<ZeroTorqueYaw<<std::endl;

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
