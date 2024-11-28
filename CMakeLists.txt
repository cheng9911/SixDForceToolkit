cmake_minimum_required(VERSION 3.0.0)
project(SixDForceTool VERSION 0.1.0 LANGUAGES C CXX)


add_subdirectory(ThirdParty/eigen-3.4.0)
add_subdirectory(ThirdParty/orocos_kinematics_dynamics-1.5.1/orocos_kdl)

add_executable(SixDForceTool main.cpp)
target_link_libraries(SixDForceTool orocos-kdl)
add_executable(test test.cpp)
target_link_libraries(test orocos-kdl)

add_executable(test1 test1.cpp)
target_link_libraries(test1 orocos-kdl)