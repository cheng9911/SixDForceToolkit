cmake_minimum_required(VERSION 3.0.0)
project(SixDForceTool VERSION 0.1.0 LANGUAGES C CXX)
find_package(orocos_kdl REQUIRED)

include_directories(include)
include_directories(ThirdParty/orocos_kinematics_dynamics-1.5.1/orocos_kdl/src)
include_directories(ThirdParty/eigen-3.4.0)
add_subdirectory(ThirdParty/eigen-3.4.0)
if(NOT TARGET orocos-kdl)
        add_subdirectory(ThirdParty/orocos_kinematics_dynamics-1.5.1/orocos_kdl)
endif()

add_executable(main src/main.cpp src/sixForceTool.cpp)
target_link_libraries(main orocos-kdl Eigen3::Eigen)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
