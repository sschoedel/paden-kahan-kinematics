cmake_minimum_required (VERSION 3.1)
project(KINEMATICS)

set(CMAKE_CXX_STANDARD 14)

include_directories (kinematics)

add_executable (6dofArm arm.cpp)

add_library (kinematics_lib STATIC kinematics/kinematics.cpp)

target_link_libraries (6dofArm LINK_PUBLIC kinematics_lib)