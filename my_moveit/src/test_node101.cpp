#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/planning_interface/planning_interface.h>



#include <moveit_visual_tools/moveit_visual_tools.h>

class goalPub1Node : public rclcpp::Node
{
public:
    goalPub1Node(const rclcpp::NodeOptions & options)
        : Node("moveit_test_pub_node",options), move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "panda_arm")
       
    {
        // Enable use_sim_time
        //setting arguments
        this->declare_parameter<bool>("use_path_constraint", false);
        bool use_path_constraint;
        this->get_parameter("use_path_constraint", use_path_constraint);
        // Set planner and pipeline

        this->declare_parameter<bool>("use_custom_planner", false);
        bool use_custom_planner;
        this->get_parameter("use_custom_planner", use_custom_planner);
        if (use_custom_planner) {
        move_group_interface_.setPlanningPipelineId("ompl");
        move_group_interface_.setPlannerId("RRTstarkConfigDefault");
        

        // Print the currently set planner and pipeline
        RCLCPP_INFO(this->get_logger(), "Current Planner ID: %s", move_group_interface_.getPlannerId().c_str());
        RCLCPP_INFO(this->get_logger(), "Current Pipeline ID: %s", move_group_interface_.getPlanningPipelineId().c_str());
        }
       
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "use_sim_time is set to true.");

        // rclcpp::executors::SingleThreadedExecutor executor;
        // executor.add_node(shared_from_this());
        // std::thread([&executor]() { executor.spin(); }).detach();

         
         // Subscriber for pose goals

        pose_subs = this->create_subscription<geometry_msgs::msg::Pose>(
            "/pose_goal", 10, std::bind(&goalPub1Node::poseCallback, this, std::placeholders::_1));

        // Subscriber for joint goals (comma-separated joint values)
        joint_subs = this->create_subscription<std_msgs::msg::String>(
            "/joint_goal", 10, std::bind(&goalPub1Node::jointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "node initialized.");

        // Store the flag for using path constraint
        if (use_path_constraint) {
            RCLCPP_INFO(this->get_logger(), "Path constraint will be used.");
        } else {
            RCLCPP_INFO(this->get_logger(), "No path constraint will be used.");
        }
        use_path_constraint_ = use_path_constraint;
    

        
    }

private:
    bool use_path_constraint_;
    //Function for pose goal with constraint
    bool planToPose(const geometry_msgs::msg::PoseStamped &target_pose)
{
    try
    {
        // Set the pose target
        move_group_interface_.setPoseTarget(target_pose.pose);
       
        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded");

            // Execute the motion
            if (move_group_interface_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Motion executed successfully.");
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Execution failed.");
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
            return false;
        }
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception during planning: %s", ex.what());
        return false;
    }
}
//callback for pose goal
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        
        RCLCPP_INFO(this->get_logger(), "got here");
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.pose = *msg;
        //move_group_interface_.setPoseTarget(*msg);
        RCLCPP_INFO(this->get_logger(), "Pose goal received.");
        //Getting data
        const auto current_pose = move_group_interface_.getCurrentPose(); 
        float xi=current_pose.pose.position.x; 
        float yi=current_pose.pose.position.y;
        float zi= current_pose.pose.position.z;
        float ri= current_pose.pose.orientation.x;
        float pi=current_pose.pose.orientation.y;
        float yawi=current_pose.pose.orientation.z;
        float wi=current_pose.pose.orientation.w;
        RCLCPP_INFO(this->get_logger(), "Current end effector position: x=%f, y=%f, z=%f, r=%f, p=%f, y=%f, w=%f", 
              xi,yi,zi,ri,pi,yawi,wi);

        bool success = planToPose(target_pose);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Pose goal achieved.");
            //rclcpp::shutdown();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Pose goal failed.");
        }
    
    }
    // For joint goal with constraint
   
    bool planToJointValues(const std::vector<double>& joint_values)
    {
    try {
        // Optional: Set a path constraint
        if (use_path_constraint_) {
            moveit_msgs::msg::Constraints path_constraints;
            RCLCPP_INFO(this->get_logger(), "I am here in my path constraints ########");
            //orientation constraint (Range limit)
            
            // moveit_msgs::msg::JointConstraint joint_constraint;
            // joint_constraint.joint_name = "panda_joint2";
            // joint_constraint.position = 0.0;          // Des
            // joint_constraint.tolerance_above = 0.5;  
            // joint_constraint.tolerance_below = 0.5;  
            // joint_constraint.weight = 1.0;
            
            // path_constraints.joint_constraints.push_back(joint_constraint);

            // Orientation Constraint
            moveit_msgs::msg::OrientationConstraint orientation_constraint;
            orientation_constraint.link_name = "panda_link8";  
            orientation_constraint.header.frame_id = "panda_link0"; 

            orientation_constraint.orientation.w = 1.0;  // no rotation
            orientation_constraint.absolute_x_axis_tolerance = 0.1;  
            orientation_constraint.absolute_y_axis_tolerance = 0.1; 
            orientation_constraint.absolute_z_axis_tolerance = 3.14; 
            orientation_constraint.weight = 1.0;

      
            path_constraints.orientation_constraints.push_back(orientation_constraint);

            move_group_interface_.setPathConstraints(path_constraints);
            RCLCPP_INFO(this->get_logger(), "Path constraint applied.");
        } else {
            // Clear all
            move_group_interface_.clearPathConstraints();
            RCLCPP_INFO(this->get_logger(), "No path constraint applied.");
        }

        move_group_interface_.setJointValueTarget(joint_values);

        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_interface_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded.");

            
            if (move_group_interface_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Motion executed successfully.");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Execution failed.");
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed.");
            return false;
        }
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(this->get_logger(), "Exception during planning: %s", ex.what());
        return false;
    }
}

//callback for Joint goal
    void jointCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::vector<double> joint_values;
        std::stringstream ss(msg->data);
        double joint_val;
        while (ss >> joint_val)
        {
            joint_values.push_back(joint_val);
            if (ss.peek() == ',')
                ss.ignore();
        }

        
        
        //move_group_interface_.setJointValueTarget(joint_values);
        RCLCPP_INFO(this->get_logger(), "Joint goal received.");

    
        //current joint values
        const auto current_joint_values = move_group_interface_.getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "Current joint values: ");
        for (size_t i = 0; i < current_joint_values.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %f", i, current_joint_values[i]);
        }

       // move_group_interface_.setPlanningTime(30.0);
        bool success = planToJointValues(joint_values);
        if (success)
        {
            // goal_reached=true;
            RCLCPP_INFO(this->get_logger(), "Joint goal executed successfully.");
            // rclcpp::shutdown();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Joint goal execution failed.");
        }
    }

    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subs;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joint_subs;
    // robot_model_loader::RobotModelLoader robot_model_loader_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    

    auto node = std::make_shared<goalPub1Node>(options);   
    // auto node = std::make_shared<goalPub1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
