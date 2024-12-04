#include <iostream>
#include "sixForceTool.h"
#include <kdl/kdl.hpp>
#include <fstream>
#include <iostream>
#define degToRad(angle) ((angle) * M_PI / 180.0)
int main(int, char**){
    
    auto *sixDForceTool = new SixDForceTool(0,0,0,0);
    int n=4;
    std::vector<SixDForce> sixDForce;
    std::vector<Pose> poses;
    sixDForce.resize(n);
    poses.resize(n);
    //弧度rad
   
    poses[0]={degToRad(146.567),degToRad(-14.478),degToRad(63.434)};
    poses[1]={degToRad(-43.894),degToRad(25.658),degToRad(56.309)};
    poses[2]={0.0,0.0,0.0};

     // poses[0]={0.0,0.0,0.0};
    // poses[1]={0.0,0.0,-KDL::PI};
    // poses[2]={0,KDL::PI/2.0,KDL::PI/2.0};
    // poses[3]={KDL::PI/2.0,-KDL::PI/2.0,0.0};
    // poses[4]={degToRad(-43.894),degToRad(25.658),degToRad(56.309)};
    // poses[5]={degToRad(146.567),degToRad(-14.478),degToRad(63.434)};
    
  
    sixDForce[0] = { 8.174,-1.498,-10.696,-0.147,-0.141,0.133};
    sixDForce[1] = { 2.787,4.174,-11.030,-189,-0.172,0.273};
    sixDForce[2] = { 6.390,2.114,-12.620,-0.251,-0.150,0.188};

    // sixDForce[0] = { 6.390,2.114,-12.620,-0.251,-0.150,0.188};
    // sixDForce[1] = { 6.424,2.029,-3.121,-0.003,-0.159,0.188};
    // sixDForce[2] = { 6.390,6.619,-7.916,-0.157,-0.155,0.188};
    // sixDForce[3] = { 6.516,-2.558,-7.850,-0.099,-0.153,0.184};
    // sixDForce[4] = { 2.787,4.174,-11.030,-189,-0.172,0.273};
    // sixDForce[5] = { 8.174,-1.498,-10.696,-0.147,-0.141,0.133};


  
    
    sixDForceTool->forces = sixDForce;
    sixDForceTool->poses = poses;

   


    sixDForceTool->LoadParameterIdentification(n);
   
    auto massResult =sixDForceTool->GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;
   

    return 0;

}
