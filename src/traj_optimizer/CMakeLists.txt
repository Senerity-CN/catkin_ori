cmake_minimum_required(VERSION 2.8.3)
project(traj_optimizer)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(Eigen3 REQUIRED)



find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    tools
    cv_bridge
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


#if this catkin packge's header is used by other packages, use catkin_package to
#declare the include directories of this package.
catkin_package(
    INCLUDE_DIRS include 
)

add_library(traj_optimizer
    src/traj_optimizer.cpp
)
target_link_libraries(traj_optimizer 
    ${catkin_LIBRARIES}
)
