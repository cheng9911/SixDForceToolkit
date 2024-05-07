cmake_minimum_required(VERSION 3.0.0)
project(SixDForceTool VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
find_library(orcos-kdl REQUIRED)
add_executable(SixDForceTool main.cpp)
target_link_libraries(SixDForceTool orocos-kdl)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
