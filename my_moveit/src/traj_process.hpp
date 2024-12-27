#ifndef TRAJ_PROCESS_HPP
#define TRAJ_PROCESS_HPP

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

namespace moveit_planner {

class TrajectorySmoothing {
public:
    void processTrajectory(robot_trajectory::RobotTrajectory& trajectory);
    double calculateTrajectoryLength(const robot_trajectory::RobotTrajectory& trajectory);
    double calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2);
    std::vector<geometry_msgs::msg::Pose> extractWaypoints(const robot_trajectory::RobotTrajectory& trajectory);
};

}  // namespace moveit_planner

#endif  // TRAJ_PROCESS_HPP