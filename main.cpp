#include <iostream>
#include "SixDForceTool.hpp"
#include <kdl/kdl.hpp>
int main(int, char**){
    std::cout << "Hello, from SixDForceTool!\n";
    double six_poses0[6] = {0.0123974,-0.575365,0.69058,-2.90244,1.20223,-1.09227e-05};
    double six_poses1[6] = {0.163479,-0.583128,0.665763,2.28035e-05,1.18628e-05,-2.35617};
    double six_poses2[6] = {-0.0496656,-0.598473,0.764768,-1.75998,0.729006,-1.76002};
    double six_poses3[6] = {0.0474273,-0.595513,0.766014,1.76,-0.729018,-1.75999};
    double six_poses4[6] = {0.191334,-0.557764,0.718021,0.613939,-1.4822,-0.61397};
    double six_poses5[6] = {-0.0102881,-0.622588,0.798573,-1.76001,-0.729027,1.75996};
    // sixDForce[0] = { -4.04905,-5.81165,-23.0022,0.188509,-0.803967,-0.0391097};
    // sixDForce[1] = { -3.23488,-4.88059,7.19731,-0.0434611,0.20479,-0.0529627};
    // sixDForce[2] = { -18.9389,-5.92461,-8.28863,0.143034,-1.14555,-0.0931772};
    // sixDForce[3] = { 11.403,-5.58718,-8.03096,0.115094,0.463372,0.00958173};
    // sixDForce[5] = { -3.68731,8.33788,-8.15823,-0.666457,-0.347675,-0.486904};
    // sixDForce[4] = { -3.82365,-21.1465,-7.96194,0.937887,-0.322435,0.412095};

    // double force_data[3] = {-6.349214527290314e-05, 0.0016341784503310919, -24.31537437438965};
    // double torque_data[3]= {-0.25042885541915894, 0.32582423090934753, 2.255179606436286e-05};
    // double euler_data[3] = {-80.50866918099089, 77.83705434751874, -9.294185889510375 + 12};
    // double force_data1[3]={-7.469202995300293, 2.3709897994995117, -23.0179500579834};
    // double torque_data1[3]= {-0.2169264256954193, 0.3719269931316376, 0.10870222747325897};
    // double euler_data1[3] = {-105.99038376663763, 60.89987226261212, -10.733422007074305 + 12};
    // double force_data2[3] = {-14.45930004119873, 0.995974063873291, -19.523677825927734};
    // double torque_data2[3]= {-0.19262456893920898, 0.3845194876194, 0.1622740775346756};
    // double euler_data2[3] = {-114.24258417090118, 43.78913507089547, -19.384088817327235 + 12};


    // double force_data1 = [-7.469202995300293, 2.3709897994995117, -23.0179500579834];
    // double torque_data1= [-0.2169264256954193, 0.3719269931316376, 0.10870222747325897];
    // double euler_data1 = [-105.99038376663763, 60.89987226261212, -10.733422007074305 + 12];

    // double force_data2 = [-14.45930004119873, 0.995974063873291, -19.523677825927734];
    // double torque_data2= [-0.19262456893920898, 0.3845194876194, 0.1622740775346756];
    // double euler_data2 = [-114.24258417090118, 43.78913507089547, -19.384088817327235 + 12];

// std::vector<SixDForce> sixDForce1;   
//     sixDForce1.resize(3);
//     sixDForce1[0] = {force_data[0],force_data[1],force_data[2],torque_data[0],torque_data[1],torque_data[2]};
//     sixDForce1[1] = {force_data1[0],force_data1[1],force_data1[2],torque_data1[0],torque_data1[1],torque_data1[2]};
//     sixDForce1[2] = {force_data2[0],force_data2[1],force_data2[2],torque_data2[0],torque_data2[1],torque_data2[2]};
//  std::vector<Pose> pose1;
//    pose1.resize(3);
//     pose1[0] = {euler_data[0]*M_PI/180,euler_data[1]*M_PI/180,euler_data[2]*M_PI/180};

//     pose1[1] = {euler_data1[0]*M_PI/180,euler_data1[1]*M_PI/180,euler_data1[2]*M_PI/180};
//     pose1[2] = {euler_data2[0]*M_PI/180,euler_data2[1]*M_PI/180,euler_data2[2]*M_PI/180};
 





    SixDForceTool sixDForceTool;
    std::vector<SixDForce> sixDForce;
    
    sixDForce.resize(6);

    sixDForce[0] = { -4.04905,-5.81165,-23.0022,0.188509,-0.803967,-0.0391097};
    sixDForce[1] = { -3.23488,-4.88059,7.19731,-0.0434611,0.20479,-0.0529627};
    sixDForce[2] = { -18.9389,-5.92461,-8.28863,0.143034,-1.14555,-0.0931772};
    sixDForce[3] = { 11.403,-5.58718,-8.03096,0.115094,0.463372,0.00958173};
    sixDForce[4] = { -3.68731,8.33788,-8.15823,-0.666457,-0.347675,-0.486904};
    sixDForce[5] = { -3.82365,-21.1465,-7.96194,0.937887,-0.322435,0.412095};
    sixDForceTool.forces = sixDForce;


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
     
    // kdl_pose[2] = KDL::Vector(1.76,-0.729018,-1.75999);
    // kdl_pose[3] = KDL::Vector(0.613939,-1.4822,-0.61397);
    // kdl_pose[5] = KDL::Vector(-1.76001,-0.729027,1.75996);

    KDL::Vector axis ;
    double norm ;
    KDL::Frame TCP_base;
    KDL::Rotation f_base;
    for(int i=0;i<6;i++){
        axis = kdl_pose[i];
        norm = axis.Normalize();
        TCP_base = KDL::Frame(KDL::Rotation::Rot(axis, norm), KDL::Vector(0, 0, 0));
        f_base = TCP_base.M*f_TCP;
        f_base.GetRPY(pose[i].roll, pose[i].pitch, pose[i].yaw);
        std::cout<<"pose[i].roll: "<<pose[i].roll<<"pose[i].pitch: "<<pose[i].pitch<<"pose[i].yaw: "<<pose[i].yaw<<std::endl;

 
        // pose[i].pitch = f_base.GetRot().y();
        // pose[i].yaw = f_base.GetRot().z();
        // sixDForceTool.addData(pose[i], sixDForce[i]);
    }

    sixDForceTool.poses = pose;

    sixDForceTool.LoadParameterIdentification(6);
   
    auto massResult =sixDForceTool.GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;
    KDL::Vector v(1.0, 2.0, 3.0);
    // Pose pose = {2.0, 2.0, 3.0};
    // SixDForce sixDForce = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    
    std::cout<< "size of poses: " << sixDForceTool.poses.size() << std::endl;
    std::cout<< "size of forces: " << sixDForceTool.forces.size() << std::endl;
    std::cout<<"poses[0].roll: "<<sixDForceTool.poses[0].roll<<std::endl;

    return 0;

}
