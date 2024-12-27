#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
    // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // // Print the default planner ID  
  // const std::string& default_planner = move_group_interface.getPlannerId();
  // RCLCPP_INFO(logger, "Current default planner ID: %s", default_planner.c_str());

  
  // // Set the desired planner ID
  // std::string planner_id = "RRTConnec";  // Example planner ID, you can change this
  // move_group_interface.setPlannerId(planner_id);
  // RCLCPP_INFO(logger, "Setting planner ID to: %s", planner_id.c_str());


  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  
  //Getting data
  const auto current_pose = move_group_interface.getCurrentPose(); 
  float xi=current_pose.pose.position.x; 
  float yi=current_pose.pose.position.y;
  float zi= current_pose.pose.position.z;
  float ri= current_pose.pose.orientation.x;
  float pi=current_pose.pose.orientation.y;
  float yawi=current_pose.pose.orientation.z;
  float wi=current_pose.pose.orientation.w;
  RCLCPP_INFO(logger, "Current end effector position: x=%f, y=%f, z=%f, r=%f, p=%f, y=%f, w=%f", 
              xi,yi,zi,ri,pi,yawi,wi);

  // Get and log the current joint states
  const auto current_joint_values = move_group_interface.getCurrentJointValues();
  RCLCPP_INFO(logger, "Current joint values: ");
  for (size_t i = 0; i < current_joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "  Joint %zu: %f", i, current_joint_values[i]);
  }

  // Set a target Pose
  //auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = ri;
    msg.orientation.y = pi;
    msg.orientation.z = yawi;
    msg.orientation.w = wi;
    msg.position.x = 0.48;
    msg.position.y = 0.0;
    msg.position.z = 0.5;
   // return msg;
  //}();
  move_group_interface.setPoseTarget(msg);//target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planing failed!");
}

  // Getting the final data
  const auto final_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(logger, "Final end effector position: x=%f, y=%f, z=%f, r=%f, p=%f, y=%f, w=%f", 
              final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z,
              final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w );

  // Get and log the final joint states
  const auto final_joint_values = move_group_interface.getCurrentJointValues();
  RCLCPP_INFO(logger, "Final joint values: ");
  for (size_t i = 0; i < final_joint_values.size(); ++i) {
    RCLCPP_INFO(logger, "  Joint %zu: %f", i, final_joint_values[i]);
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}