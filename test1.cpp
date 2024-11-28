#include <iostream>
#include "SixDForceTool.hpp"
#include <kdl/kdl.hpp>
#define deg2rad M_PI / 180.0
int main(int, char **)
{
    std::cout << "Hello, from SixDForceTool!\n";
    // 采集的法兰盘姿态
    double six_poses0[6] = {0, 0, 0, -80.50866918099089*deg2rad, 77.83705434751874*deg2rad, (-9.294185889510375 + 12)*deg2rad};
    double six_poses1[6] = {0, 0, 0, -105.99038376663763*deg2rad, 60.89987226261212*deg2rad, (-10.733422007074305 + 12)*deg2rad};
    double six_poses2[6] = {0, 0, 0, -114.24258417090118*deg2rad, 43.78913507089547*deg2rad, (-19.384088817327235 + 12)*deg2rad};
    

    SixDForceTool sixDForceTool;
    std::vector<SixDForce> sixDForce;

    sixDForce.resize(3);
    // 采集的六维力数据，数据sixDForce[0]对应six_poses0[6]姿态下。
    sixDForce[0] = {-6.349214527290314e-05, 0.0016341784503310919, -24.31537437438965, -0.25042885541915894, 0.32582423090934753, 2.255179606436286e-05};
    sixDForce[1] = {-7.469202995300293, 2.3709897994995117, -23.0179500579834, -0.2169264256954193, 0.3719269931316376, 0.10870222747325897};
    sixDForce[2] = {-14.45930004119873, 0.995974063873291, -19.523677825927734, -0.19262456893920898, 0.3845194876194, 0.1622740775346756};

    // 传入六维力数据
    sixDForceTool.forces = sixDForce;

    // 六维力到法兰盘的转换矩阵


    std::vector<Pose> pose;

    pose.resize(3);
  
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
    

    // 传入位置姿态
    sixDForceTool.poses = pose;
    // 负载参数辩识
    sixDForceTool.LoadParameterIdentification(3);
    // 获取质量和重力参数
    auto massResult = sixDForceTool.GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;

    return 0;
}
