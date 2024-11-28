#include <iostream>
#include "SixDForceTool.hpp"
#include <kdl/kdl.hpp>
#define deg2rad M_PI / 180.0
int main(int, char **)
{
    std::cout << "Hello, from SixDForceTool!\n";
    // 采集的法兰盘姿态
    double six_poses0[6] = {0, 0, 0, 180 * deg2rad, 0, 90 * deg2rad};
    double six_poses1[6] = {0, 0, 0, -153.2060 * deg2rad, 37.7609 * deg2rad, 102.1478 * deg2rad};
    double six_poses2[6] = {0, 0, 0, -98.59 * deg2rad, -18.4609 * deg2rad, 85.7497 * deg2rad};
    double six_poses3[6] =  {0, 0, 0, -157.6449 * deg2rad, 24.5498 * deg2rad, 103.9308 * deg2rad};
    // double six_poses3[6] = {0, 0, 0, -126.4858 * deg2rad, -34.6656 * deg2rad, 51.5642 * deg2rad};
    double six_poses4[6] = {0, 0, 0, -108.9637 * deg2rad, -15.1274 * deg2rad, 95.5084 * deg2rad};
    // double six_poses4[6] = {0, 0, 0, -157.6449 * deg2rad, 24.5498 * deg2rad, 103.9308 * deg2rad};
    double six_poses5[6] = {0, 0, 0, -108.9637 * deg2rad, -15.1274 * deg2rad, 95.5084 * deg2rad};

    SixDForceTool sixDForceTool;
    std::vector<SixDForce> sixDForce;

    sixDForce.resize(6);
    // 采集的六维力数据，数据sixDForce[0]对应six_poses0[6]姿态下。
    sixDForce[0] = {7.49, 3.17, 6.04, 0.03, -0.36, -0.01};
    sixDForce[1] = {12.59, 6.08, 3.62, -0.09, -0.13, 0.02};
    sixDForce[2] = {5.08, 10.69, -1.14, -0.3, -0.46, -0.01};
    sixDForce[3] = {3.10, 8.78, 7.92, -0.2, -0.57, -0.01};
    sixDForce[4] = {11.02, 6.06, 4.85, -0.09, -0.21, 0.01};
    sixDForce[5] = {5.35, 10.69, 0.28, -0.29, -0.44, 0.00};
    // 传入六维力数据
    sixDForceTool.forces = sixDForce;

    // 六维力到法兰盘的转换矩阵


    std::vector<Pose> pose;

    pose.resize(6);
  
    // 公式变换
    pose[0].roll = six_poses0[3];
    pose[0].pitch = six_poses0[4];
    pose[0].yaw = six_poses0[5];
    pose[1].roll = six_poses1[3];
    pose[1].pitch = six_poses1[4];
    pose[1].yaw = six_poses1[5];
    pose[2].roll = six_poses2[3];
    pose[2].pitch = six_poses2[4];
    pose[2].yaw = six_poses2[5];
    pose[3].roll = six_poses3[3];
    pose[3].pitch = six_poses3[4];
    pose[3].yaw = six_poses3[5];
    pose[4].roll = six_poses4[3];
    pose[4].pitch = six_poses4[4];
    pose[4].yaw = six_poses4[5];
    pose[5].roll = six_poses5[3];
    pose[5].pitch = six_poses5[4];
    pose[5].yaw = six_poses5[5];

    // 传入位置姿态
    sixDForceTool.poses = pose;
    // 负载参数辩识
    sixDForceTool.LoadParameterIdentification(3);
    // 获取质量和重力参数
    auto massResult = sixDForceTool.GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;

    return 0;
}
