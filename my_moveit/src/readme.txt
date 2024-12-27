Commands:

ros2 topic pub /joint_goal std_msgs/msg/String "data: '-1.86, -1.46, 0.990, -2.04,1.18,1.116,-0.10'"

ros2 topic pub /pose_goal geometry_msgs/msg/Pose "{position: {x: 0.5, y: 0.0, z: 0.590}}"


 colcon build --packages-select my_moveit --symlink-install


ros2 run my_moveit test_node101 --ros-args -p use_custom_planner:=true
ros2 run my_moveit test_node101 --ros-args -p use_path_constraint:=true
