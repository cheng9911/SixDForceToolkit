#include <iostream>
#include "SixDForceTool.hpp"
#include <kdl/kdl.hpp>
int main(int, char**){
    std::cout << "Hello, from SixDForceTool!\n";
    SixDForceTool sixDForceTool(10.0, 1.0, 2.0, 3.0);
    auto massResult =sixDForceTool.GetMassAndGravity();
    std::cout << "mass: " << massResult.mass << ", massx: " << massResult.massx << ", massy: " << massResult.massy << ", massz: " << massResult.massz << std::endl;
    KDL::Vector v(1.0, 2.0, 3.0);
    Pose pose = {2.0, 2.0, 3.0};
    SixDForce sixDForce = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    sixDForceTool.addData(pose, sixDForce);
    std::cout<< "size of poses: " << sixDForceTool.poses.size() << std::endl;
    std::cout<< "size of forces: " << sixDForceTool.forces.size() << std::endl;
    std::cout<<"poses[0].roll: "<<sixDForceTool.poses[0].roll<<std::endl;

    return 0;

}
