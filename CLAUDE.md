# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS catkin workspace implementing a mobile robot planning and control system that includes:
- Hybrid A* kinodynamic planning for collision-free path search
- Trajectory optimization with obstacle avoidance constraints
- Real-time replanning capability for dynamic environments
- Integration with sensor data (LIDAR, odometry, occupancy grid maps)

## Build System

The project uses standard catkin build system:

**To build:**
```bash
cd /path/to/catkin_ori
catkin_make
# or for faster builds
catkin build
```

**Dependencies:**
- ROS Noetic
- Eigen3
- PCL (Point Cloud Library)
- OMPL (Open Motion Planning Library)
- Torch (PyTorch)

## Run Commands

**To run the complete system:**
```bash
# Launch everything using the provided script
./run.sh

# Or launch individual components:
roslaunch plan_manage plan_node.launch
roslaunch random_map_generator test.launch
```

**Individual package nodes:**
- `plan_node` - Main planning system (path search + optimization)
- `laser_node` - Point cloud processing
- `controller_node` - Trajectory following (disabled by default)

## Package Architecture

### plan_manage (main package)
Contains the core planning logic:
- `plan_manager.cpp/h`: Main orchestration including odometry and target callbacks
- Real-time replanning with collision detection
- Integration of front-end path search with back-end optimization
- Interfaces with trajectory optimization module

### path_search
Contains the kinodynamic path search algorithm:
- KinoAstar - Hybrid A* implementation with motion constraints
- Dubins/Reeds-Shepp curves for non-holonomic robots
- Handles velocity and acceleration constraints

### traj_optimizer
Trajectory optimization using L-BFGS solver:
- Minimizes jerk trajectories with obstacle avoidance
- Maintains safety constraints via polygonal environment representations
- Handles time optimization with variable segment durations

### utils/tools
Various utilities:
- Grid map utilities for environment representation
- Visualization tools
- Configuration managers

## Important Development Notes

**Key Topics:**
- `/move_base_simple/goal` - target pose commands
- `/ugv/odometry` - robot pose (replace as needed for your robot)
- `/ugv/laser` or `/point_cloud` - sensor data
- `/map` - occupancy grid map

**Configuration:**
- System parameters in `src/plan_manage/config/planning.yaml`
- Global config in `src/plan_manage/config/global_config.yaml`

**Key Classes:**
- `PlanManager` - top-level planning coordination
- `KinoAstar` - path search with motion constraints
- `PolyTrajOptimizer` - trajectory optimization with obstacle avoidance