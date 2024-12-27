#include "traj_process.hpp"
#include <cmath>

namespace moveit_planner {

void TrajectorySmoothing::processTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
    double original_length = calculateTrajectoryLength(trajectory);
    std::vector<geometry_msgs::msg::Pose> waypoints = extractWaypoints(trajectory);

    // Example shortcutting algorithm
    for (int i = 0; i < 100; ++i)  // Loop 100 times for shortcutting
    {
        int idx1 = rand() % waypoints.size();
        int idx2 = rand() % waypoints.size();

        // if (idx1 != idx2 && calculateDistance(waypoints[idx1], waypoints[idx2]) < 0.5)
        // {
        //     waypoints[idx1] = waypoints[idx2];  // Shortcut
        // }
        // if (idx1 != idx2 && isCollisionFree(waypoints[idx1], waypoints[idx2]))
        // {
        //     //do something
        // }

    }

    double smoothed_length = calculateTrajectoryLength(trajectory);
    if (smoothed_length < original_length)
    {
        RCLCPP_INFO(rclcpp::get_logger("TrajectorySmoothing"), "Smoothed trajectory is shorter: %f", smoothed_length);
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("TrajectorySmoothing"), "Original trajectory is already optimal.");
    }
}

double TrajectorySmoothing::calculateDistance(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2)
{
    double dx = pose2.position.x - pose1.position.x;
    double dy = pose2.position.y - pose1.position.y;
    double dz = pose2.position.z - pose1.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double TrajectorySmoothing::calculateTrajectoryLength(const robot_trajectory::RobotTrajectory& trajectory)
{
    double length = 0.0;
    for (size_t i = 1; i < trajectory.getWayPointCount(); ++i)
    {
        const auto& prev_state = trajectory.getWayPoint(i - 1);
        const auto& curr_state = trajectory.getWayPoint(i);
        length += prev_state.distance(curr_state);  // Calculate Euclidean distance
    }
    return length;
}

std::vector<geometry_msgs::msg::Pose> TrajectorySmoothing::extractWaypoints(const robot_trajectory::RobotTrajectory& trajectory)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
    {
        const auto& state = trajectory.getWayPoint(i);
        geometry_msgs::msg::Pose pose;

        // Extract pose
        auto transform = state.getGlobalLinkTransform("panda_link8");
        pose.position.x = transform.translation().x();
        pose.position.y = transform.translation().y();
        pose.position.z = transform.translation().z();

        waypoints.push_back(pose);
    }
    return waypoints;
}

// bool TrajectorySmoothing::isCollisionFree(const geometry_msgs::msg::Pose& pose1, const geometry_msgs::msg::Pose& pose2, 
//                                           moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
// {
//     // Create a temporary trajectory with a line between the two poses
//     std::vector<geometry_msgs::msg::Pose> line_poses;
//     line_poses.push_back(pose1);
//     line_poses.push_back(pose2);

//     // Set up a collision request
//     collision_detection::CollisionRequest collision_request;
//     collision_request.contacts = false;  // Disable contact details to save computation
//     collision_request.max_contacts = 0;  // We only care whether there is a collision, not the details

//     // Create a CollisionResult to store the results
//     collision_detection::CollisionResult collision_result;

//     // Set up a PlanningScene for collision checking
//     planning_scene::PlanningScene planning_scene(move_group_interface.getRobotModel());
//     planning_scene.checkCollision(collision_request, collision_result);

//     // Check if the trajectory between pose1 and pose2 is valid (no collision)
//     return !collision_result.collision;
// }


}  // namespace moveit_planner