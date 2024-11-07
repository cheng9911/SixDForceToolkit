#include <iostream>
#include "SixDForceTool.hpp"
#include <kdl/kdl.hpp>
int main(int, char**){
    std::cout << "Hello, from SixDForceTool!\n";
    // 采集的法兰盘姿态
    double six_poses0[6] = {0.0123974,-0.575365,0.69058,-2.90244,1.20223,-1.09227e-05};
    double six_poses1[6] = {0.163479,-0.583128,0.665763,2.28035e-05,1.18628e-05,-2.35617};
    double six_poses2[6] = {-0.0496656,-0.598473,0.764768,-1.75998,0.729006,-1.76002};
    double six_poses3[6] = {0.0474273,-0.595513,0.766014,1.76,-0.729018,-1.75999};
    double six_poses4[6] = {0.191334,-0.557764,0.718021,0.613939,-1.4822,-0.61397};
    double six_poses5[6] = {-0.0102881,-0.622588,0.798573,-1.76001,-0.729027,1.75996};

    SixDForceTool sixDForceTool;
    std::vector<SixDForce> sixDForce;
    
    sixDForce.resize(6);
    // 采集的六维力数据，数据sixDForce[0]对应six_poses0[6]姿态下。
    sixDForce[0] = { -4.04905,-5.81165,-23.0022,0.188509,-0.803967,-0.0391097};
    sixDForce[1] = { -3.23488,-4.88059,7.19731,-0.0434611,0.20479,-0.0529627};
    sixDForce[2] = { -18.9389,-5.92461,-8.28863,0.143034,-1.14555,-0.0931772};
    sixDForce[3] = { 11.403,-5.58718,-8.03096,0.115094,0.463372,0.00958173};
    sixDForce[4] = { -3.68731,8.33788,-8.15823,-0.666457,-0.347675,-0.486904};
    sixDForce[5] = { -3.82365,-21.1465,-7.96194,0.937887,-0.322435,0.412095};
    // 传入六维力数据
    sixDForceTool.forces = sixDForce;

    // 六维力到法兰盘的转换矩阵
    KDL::Rotation f_TCP = KDL::Rotation::RPY(M_PI,0,-M_PI/4); 

    std::vector<Pose> pose;
    std::vector<KDL::Vector> kdl_pose;
    pose.resize(6);
    kdl_pose.resize(6);
    kdl_pose[0] = KDL::Vector(six_poses0[3],six_poses0[4],six_poses0[5]);
    kdl_pose[1] = KDL::Vector(six_poses1[3],six_poses1[4],six_poses1[5]);
    kdl_pose[2] = KDL::Vector(six_poses2[3],six_poses2[4],six_poses2[5]);
    kdl_pose[3] = KDL::Vector(six_poses3[3],six_poses3[4],six_poses3[5]);
    kdl_pose[4] = KDL::Vector(six_poses4[3],six_poses4[4],six_poses4[5]);
    kdl_pose[5] = KDL::Vector(six_poses5[3],six_poses5[4],six_poses5[5]);
    KDL::Vector axis ;
    double norm ;
    KDL::Frame TCP_base;
    KDL::Rotation f_base;
    // 公式变换
    for(int i=0;i<6;i++){
        axis = kdl_pose[i];
        norm = axis.Normalize();
        TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(0, 0, 0));
        f_base = TCP_base.M*f_TCP;
        // 转换为RPY描述的姿态
        f_base.GetRPY(pose[i].roll, pose[i].pitch, pose[i].yaw);
        // std::cout<<"pose[i].roll: "<<pose[i].roll<<"pose[i].pitch: "<<pose[i].pitch<<"pose[i].yaw: "<<pose[i].yaw<<std::endl;
    }
    // 传入位置姿态
    sixDForceTool.poses = pose;
    // 负载参数辩识
    sixDForceTool.LoadParameterIdentification(6);
    // 获取质量和重力参数
    auto massResult =sixDForceTool.GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;


    return 0;

}
