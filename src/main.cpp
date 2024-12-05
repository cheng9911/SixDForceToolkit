#include <iostream>
#include "sixForceTool.h"
#include <kdl/kdl.hpp>
#include <fstream>
#include <iostream>
#define degToRad(angle) ((angle) * M_PI / 180.0)
int main(int, char**){
    
    auto *sixDForceTool = new SixDForceTool(0,0,0,0);
    int n=6;
    std::vector<SixDForce> sixDForce;
    std::vector<Pose> poses;
    sixDForce.resize(n);
    poses.resize(n);
    //弧度rad
   
    

    poses[0]={0.0,0.0,0.0};
    poses[1]={0.0,0.0,-KDL::PI};
    poses[2]={0,KDL::PI/2.0,KDL::PI/2.0};
    poses[3]={KDL::PI/2.0,-KDL::PI/2.0,0.0};
    poses[4]={degToRad(-43.894),degToRad(25.658),degToRad(56.309)};
    poses[5]={degToRad(146.567),degToRad(-14.478),degToRad(63.434)};
    
  
    

    sixDForce[0] = { 6.39,2.114,-12.620,-0.251,-0.148,0.188};
    sixDForce[1] = { 6.424,2.029,-3.121,-0.003,-0.159,0.188};
    sixDForce[2] = { 6.390,6.619,-7.916,-0.157,-0.155,0.188};
    sixDForce[3] = { 6.516,-2.558,-7.850,-0.099,-0.153,0.184};
    sixDForce[4] = { 2.787,4.174,-11.030,-0.189,-0.172,0.273};
    sixDForce[5] = { 8.174,-1.498,-10.696,-0.147,-0.141,0.133};

    
    std::vector<KDL::Vector> kdl_pose;
    kdl_pose.resize(6);
    kdl_pose[0] = KDL::Vector(poses[0].roll, poses[0].pitch, poses[0].yaw);
    kdl_pose[1] = KDL::Vector(poses[1].roll, poses[1].pitch, poses[1].yaw);
    kdl_pose[2] = KDL::Vector(poses[2].roll, poses[2].pitch, poses[2].yaw);
    kdl_pose[3] = KDL::Vector(poses[3].roll, poses[3].pitch, poses[3].yaw);
    kdl_pose[4] = KDL::Vector(poses[4].roll, poses[4].pitch, poses[4].yaw);
    kdl_pose[5] = KDL::Vector(poses[5].roll, poses[5].pitch, poses[5].yaw);
    KDL::Vector axis;
    double norm;

  
     // 六维力到法兰盘的转换矩阵
    KDL::Rotation f_TCP = KDL::Rotation::RPY(0, 0, M_PI);
    //公式变换
    for (int i = 0; i < n; i++)
    {
        KDL::Rotation rotation =KDL::Rotation::RotX(poses[i].yaw)*KDL::Rotation::RotY(poses[i].pitch)*KDL::Rotation::RotZ(poses[i].roll);
     
        KDL::Frame TCP_base = KDL::Frame(rotation, KDL::Vector(0, 0, 0));
        KDL::Rotation f_base = TCP_base.M * f_TCP;
        // 转换为RPY描述的姿态
        f_base.GetRPY(poses[i].roll, poses[i].pitch, poses[i].yaw);
        // std::cout<<"pose[i].roll: "<<pose[i].roll<<"pose[i].pitch: "<<pose[i].pitch<<"pose[i].yaw: "<<pose[i].yaw<<std::endl;
    }
   


    sixDForceTool->forces = sixDForce;
    sixDForceTool->poses = poses;

   


    sixDForceTool->LoadParameterIdentification(n);
   
    auto massResult =sixDForceTool->GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;
   

    return 0;

}
