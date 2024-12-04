cmake_minimum_required(VERSION 3.0.0)
project(SixDForceTool VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
find_library(orcos-kdl REQUIRED)
find_package(Eigen3 REQUIRED)
add_executable(main src/main.cpp src/sixForceTool.cpp)
target_link_libraries(main orocos-kdl)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
