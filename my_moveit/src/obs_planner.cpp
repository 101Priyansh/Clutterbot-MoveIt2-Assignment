#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "traj_process.hpp"

namespace moveit_planner {

class PlanwithObstacle {
public:
    explicit PlanwithObstacle(rclcpp::Node::SharedPtr node)
        : node_(node), arm_group_interface_(node, "panda_arm"), visual_tools_(node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, arm_group_interface_.getRobotModel()) 
    {
        arm_group_interface_.setPlanningPipelineId("ompl");
        arm_group_interface_.setPlannerId("RRTConnectkConfigDefault");
        arm_group_interface_.setPlanningTime(1.0);
        arm_group_interface_.setMaxVelocityScalingFactor(1.0);
        arm_group_interface_.setMaxAccelerationScalingFactor(1.0);


        node_->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(node_->get_logger(), "use_sim_time is set to true.");
        
        

        // MoveItVisualTools
        visual_tools_.deleteAllMarkers();
        visual_tools_.loadRemoteControl();
    }

    void setTargetPose(const geometry_msgs::msg::PoseStamped &target_pose)
    {
        arm_group_interface_.setPoseTarget(target_pose);
    }

    bool planAndExecute(moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        bool success = static_cast<bool>(arm_group_interface_.plan(plan));
        if (success)
        {
            arm_group_interface_.execute(plan);
            //visualizeEndEffector(plan);
        }
        return success;
    }
    // To add object in rviz
    void addObstacle(const geometry_msgs::msg::Pose &pose, double radius, const std::string &id)
    {
        RCLCPP_INFO(rclcpp::get_logger("ObstacleHandler"), "Adding obstacle: %s", id.c_str());

        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = arm_group_interface_.getPlanningFrame();
        collision_object.header.stamp = node_->now();
        collision_object.id = id;

        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions = {radius};

        collision_object.primitives.push_back(sphere);
        collision_object.primitive_poses.push_back(pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        planning_scene_interface_.applyCollisionObject(collision_object);

        //  obstacle
        visual_tools_.publishSphere(pose, rviz_visual_tools::RED, rviz_visual_tools::XLARGE, "Obstacle: " + id);
    }
// To remove object in rviz
    void removeObstacle(const std::string &id)
    {
        std::vector<std::string> object_ids = {id};
        planning_scene_interface_.removeCollisionObjects(object_ids);

        visual_tools_.deleteAllMarkers();
        visual_tools_.trigger();

        RCLCPP_INFO(rclcpp::get_logger("ObstacleHandler"), "Obstacle removed: %s", id.c_str());
    }

    // void visualizeEndEffector(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    // {
    //     const auto current_pose = arm_group_interface_.getCurrentPose("panda_hand"); 
    //     geometry_msgs::msg::Pose pose;
    //     pose.position.x=current_pose.pose.position.x; 
    //     pose.position.y=current_pose.pose.position.y;
    //     pose.position.z= current_pose.pose.position.z;
    //     pose.orientation.w = 1.0;  // Assuming the orientation doesn't need to change

    //     visual_tools_.publishSphere(pose, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, "EndEffector");
    //     visual_tools_.trigger();
    // }

    moveit::planning_interface::MoveGroupInterface& getMoveGroupInterface()
    {
        return arm_group_interface_;
    }
//To shorten the trajectory
    void processTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory) {
        auto &points = trajectory.joint_trajectory.points;
        // RCLCPP_INFO(node_->get_logger(), "Original Trajectory Waypoints:");
        // for (const auto &point : points) {
        //     RCLCPP_INFO(node_->get_logger(), "Positions: [%f, %f, %f]", point.positions[0], point.positions[1], point.positions[2]);
        //}
        for (int i = 0; i < 100; ++i) {
            int idx1 = rand() % points.size();
            int idx2 = rand() % points.size();
            if (idx1 != idx2 && isCollisionFree(points[idx1], points[idx2])) {
                points.erase(points.begin() + std::min(idx1, idx2) + 1, points.begin() + std::max(idx1, idx2));
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Trajectory shorteninh completed.");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;  // Set the modified trajectory
        auto &point_ss=trajectory.joint_trajectory.points;

        if (points.size()>point_ss.size())
        {
            RCLCPP_INFO(node_->get_logger(), "Path is actually Shorter!");
        }

        
        if (planAndExecute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Executed the shorter trajectory successfully.");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to execute the shorter trajectory.");
        }
    }

    bool isCollisionFree(const trajectory_msgs::msg::JointTrajectoryPoint &p1, const trajectory_msgs::msg::JointTrajectoryPoint &p2) {
        
    // Get the robot model and current state
    const moveit::core::RobotModelConstPtr& kinematic_model = arm_group_interface_.getRobotModel();  
    planning_scene::PlanningScene planning_scene(kinematic_model);

    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
    // RCLCPP_INFO(node_->get_logger(), "inside Collision checking.");
    
    const int num_intermediate_points = 10;  // number of intermediate points
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> trajectory_points;

    // Add the starting point p1
    trajectory_points.push_back(p1);

    // Interpolate between p1 and p2 to create intermediate points
    for (int i = 1; i < num_intermediate_points; ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint intermediate_point;
        for (size_t j = 0; j < p1.positions.size(); ++j) {
            // Linear interpolation between joint positions of p1 and p2
            double interpolated_position = p1.positions[j] + (p2.positions[j] - p1.positions[j]) * (i / static_cast<double>(num_intermediate_points));
            intermediate_point.positions.push_back(interpolated_position);
        }
        trajectory_points.push_back(intermediate_point);
    }

    // Add the final point p2
    trajectory_points.push_back(p2);

    // Set up the collision request
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;  // Get contact information
    collision_request.max_contacts = 100;

    collision_detection::CollisionResult collision_result;
    //RCLCPP_INFO(node_->get_logger(), "Checking Collision.");
    // Check for collisions along the path
    for (const auto& point : trajectory_points) {
        // Set the joint positions to the current point 
        current_state.setJointGroupPositions(joint_model_group, point.positions);

        // check collision 
        planning_scene.checkCollision(collision_request, collision_result, current_state, planning_scene.getAllowedCollisionMatrix());

        // If collision: detected, return false 
        if (collision_result.collision) {
            return false;
        }
    }

    // If collisions : not detected, return true
    return true;
    RCLCPP_INFO(node_->get_logger(), "Collision checked.");
     }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface arm_group_interface_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

}  // namespace moveit_planner

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("plan_around_objects");
    
    moveit_planner::PlanwithObstacle move_group_manager(node);

    // target pose
    geometry_msgs::msg::PoseStamped arm_target_pose;
    arm_target_pose.header.frame_id = "base_link";
    arm_target_pose.header.stamp = node->now();
    arm_target_pose.pose.position.x = 0.128;
    arm_target_pose.pose.position.y = -0.266;
    arm_target_pose.pose.position.z = 0.111;
    arm_target_pose.pose.orientation.x = 0.635;
    arm_target_pose.pose.orientation.y = -0.268;
    arm_target_pose.pose.orientation.z = 0.694;
    arm_target_pose.pose.orientation.w = 0.206;

    // home pose
    // geometry_msgs::msg::PoseStamped arm_home_pose;
    // arm_home_pose.header.frame_id = "base_link";
    // arm_home_pose.header.stamp = node->now();
    // arm_home_pose.pose.position.x = 0.307020;
    // arm_home_pose.pose.position.y = -0.00000;
    // arm_home_pose.pose.position.z = 0.590270;
    // arm_home_pose.pose.orientation.x = 0.923956;
    // arm_home_pose.pose.orientation.y = -0.382499;
    // arm_home_pose.pose.orientation.z = 0.00000;
    // arm_home_pose.pose.orientation.w = 0.00000;


    move_group_manager.setTargetPose(arm_target_pose);

    // obstacle pose
    geometry_msgs::msg::Pose obstacle_pose;
    obstacle_pose.position.x = 0.29;
    obstacle_pose.position.y = -0.19;
    obstacle_pose.position.z = 0.37;
    obstacle_pose.orientation.w = 1.0;
    move_group_manager.addObstacle(obstacle_pose, 0.1, "obstacle1");
    
    // Plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan avoid_plan;
    bool success = static_cast<bool>(move_group_manager.getMoveGroupInterface().plan(avoid_plan));
    //bool success = move_group_manager.planAndExecute(avoid_plan);

    if (success)
    {   
        //##plan without obstacle##//

        // move_group_manager.removeObstacle("obstacle1");
        // bool success = move_group_manager.planAndExecute(plan);
        // if (success)
        // {
        //     RCLCPP_INFO(rclcpp::get_logger("plan_around_objects"), "Moving on to Post processing");
        // }

        //##Sending back to home##//
        // moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        // move_group_manager.setTargetPose(arm_home_pose);
        // //move_group_manager.getMoveGroupInterface().setStartStateToCurrentState();
        // move_group_manager.getMoveGroupInterface().setPlanningTime(10.0);
        // RCLCPP_INFO(rclcpp::get_logger("plan_around_objects"), "Back to Home");
        // bool success = move_group_manager.planAndExecute(home_plan);

        //if (success){

        RCLCPP_INFO(rclcpp::get_logger("plan_around_objects"), "inside the Plan with Obstacle Node...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        move_group_manager.processTrajectory(avoid_plan.trajectory_);
        //}
    }

    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();


    rclcpp::shutdown();
    return success ? 0 : 1;
}

