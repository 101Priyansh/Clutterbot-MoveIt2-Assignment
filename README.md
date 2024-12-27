# Clutterbot-MoveIt2-Assignment

## Instructions:

Location: Inside Moveit2 tutorial repository.
cd ~/ws_moveit/src/my_moveit


## Build:

cd ~/ws_moveit/
colcon build --mixin release

colcon build --packages-select my_moveit

## Description of scripts:
1. test_node101.cpp: Sends Pose goal and Joint goal using custom planners and with path constraints. Conatins Subscriber for Pose goal and Joint Goal along with PoseToPlan and Joint Function with Custom Planner Loader and Path cnstraints. arguments: use_custom_planner,use_path_constraint.
2. obs_planner.cpp: Plans for pose with and without obstacle and executes the shorter trajectory. Contains Obstacle addition and removing functions and Trajectory Processing function.

## Commands:

### Part 1: Initialisation and setup of Pose goal and Joint goal topics.

To launch Move_group:
ros2 launch ws_moveit2 demo.launch.py (using the tutorial's launch file).

To Start node with default planner and without constraints:
ros2 run my_moveit test_node101

To Publish to Pose Goal:
ros2 topic pub --once /pose_goal geometry_msgs/msg/Pose "{position: {x: 0.5, y: 0.0, z: 0.590}}"

To Publish to Joint Goal:
ros2 topic pub --once /joint_goal std_msgs/msg/String "data: '-1.86, -1.46, 0.990, -2.04,1.18,1.116,-0.10'"

### Part 2: Functions for Pose goal and Joint goal.

To launch Move_group:
ros2 launch ws_moveit2 demo.launch.py (using the tutorial's launch file).

### Part 2.1: To Start node with custom planner and without constraints:
ros2 run my_moveit test_node101 --ros-args -p use_custom_planner:=true
![Parameter set to specific Planner](result/image.png)

To Publish to Pose Goal:
ros2 topic pub --once /pose_goal geometry_msgs/msg/Pose "{position: {x: 0.5, y: 0.0, z: 0.590}}"

### Part 2.2: To Start node without custom planner and with constraints:
ros2 run my_moveit test_node101 --ros-args -p use_path_constraint:=true
![Using Constraint](result/image.png)
To Publish to Joint Goal:
ros2 topic pub --once /joint_goal std_msgs/msg/String "data: '-1.86, -1.46, 0.990, -2.04,1.18,1.116,-0.10'"
![Failed to Plan](result/image.png)
### Part 3: Planning with and without obstacle & Trajectory Post processing:
To launch Move_group:
ros2 launch ws_moveit2 demo.launch.py (using the tutorial's launch file).

To add obstacle, plan, with and without obstacle and post processing:
ros2 run my_moveit obs_planner

**GIFs:**
1. Adding and Removing Obstacles  
   ![obstacle in Rviz](path/to/your/add_obstacles.gif)
2. Trajectory Post-Processing   
   ![Trajectory 1](path/to/your/plan_without_obstacles.gif)   ![Trajectory 2](path/to/your/trajectory_post_processing.gif)

