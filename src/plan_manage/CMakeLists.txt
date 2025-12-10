# cmake_minimum_required(VERSION 3.5) # 建议提高版本
# project(plan_manage)

# set(CMAKE_VERBOSE_MAKEFILE "true")
# # CMAKE_BUILD_TYPE 最好在编译时通过 catkin_make -D... 指定，或通过 profile

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

# add_compile_options(-Wall -fPIC) # -g 会自动为Debug添加
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0") # Debug时通常不需要高优化

# find_package(Eigen3 REQUIRED)
# find_package(Torch REQUIRED) # 确保你的FindTorch.cmake能正确找到并设置好一切
# find_package(PCL REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     visualization_msgs
#     std_msgs
#     # tools # 如果 tools 是一个需要 find_package 的包，应该也在这里
#     path_search # 假设是另一个catkin包或已正确find_package
#     cv_bridge
#     decomp_ros_utils # 假设是另一个catkin包或已正确find_package
#     traj_optimizer # 假设是另一个catkin包或已正确find_package
# )

# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIRS}
#     ${PCL_INCLUDE_DIRS}
#     ${TORCH_INCLUDE_DIRS} # 如果不使用 Torch::torch 目标链接，可能需要这行
# )

# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES plan_manage # 如果 plan_manage 库需要被其他包使用
#     CATKIN_DEPENDS roscpp std_msgs # 等等运行时依赖
#     DEPENDS Eigen3 PCL Torch # 如果公共头文件用了这些，或者需要传递依赖
# )

# add_library(plan_manage
#     src/plan_manage.cpp
# )
# target_link_libraries(plan_manage PRIVATE # 或者 PUBLIC，取决于你的设计
#     kino_astar # 确保这些目标存在
#     traj_optimizer # 确保这些目标存在
#     ${catkin_LIBRARIES}
#     Eigen3::Eigen # 现代Eigen链接方式
#     ${PCL_LIBRARIES} # 示例，具体PCL模块根据需要
#     # Torch::torch # 如果 plan_manage 库也用到了Torch
# )

# add_executable(plan_node
#     src/plan_node.cpp
# )
# target_link_libraries(plan_node PRIVATE
#     plan_manage
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )

# # add_executable(controller_node
# #   src/rlcontroller.cpp
# # )
# # target_link_libraries(controller_node PRIVATE
# #   ${catkin_LIBRARIES}
# #   ${TORCH_LIBRARIES}
# # )

# add_executable(laser_node
#   src/laser.cpp
# )
# target_link_libraries(laser_node PRIVATE
#   ${catkin_LIBRARIES}
# )

cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

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
    path_search
    cv_bridge
    decomp_ros_utils
    traj_optimizer
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

add_library(plan_manage
    src/plan_manage.cpp
    src/trajectory_metrics.cpp
)
target_link_libraries(plan_manage 
    kino_astar
    traj_optimizer
    ${catkin_LIBRARIES}
)
add_executable(plan_node
    src/plan_node.cpp
)
target_link_libraries(plan_node 
    plan_manage
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
# add_executable(vis_node
#     src/visualize_node.cpp
# )
# target_link_libraries(vis_node 
#     ${catkin_LIBRARIES}
#     ${PCL_LIBRARIES}
# )

add_executable(controller_node
  src/controller.cpp
  # src/rlcontroller.cpp
)

target_link_libraries(controller_node 
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
)

add_executable(laser_node
  src/laser.cpp
)

target_link_libraries(laser_node 
  ${catkin_LIBRARIES}
)