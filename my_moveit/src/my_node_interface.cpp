#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

class goalPubNode : public rclcpp::Node
{
public:
    goalPubNode()
        : Node("moveit_goal_pub_node"), move_group_interface_(std::shared_ptr<rclcpp::Node>(this), "panda_arm")
       
    {
        // Enable use_sim_time
       
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));
        RCLCPP_INFO(this->get_logger(), "use_sim_time is set to true.");

        // rclcpp::executors::SingleThreadedExecutor executor;
        // executor.add_node(shared_from_this());
        // std::thread spinner = std::thread([&executor]() { executor.spin(); });
        //  // Subscriber for pose goals

        pose_subs = this->create_subscription<geometry_msgs::msg::Pose>(
            "/pose_goal", 10, std::bind(&goalPubNode::poseCallback, this, std::placeholders::_1));

        // Subscriber for joint goals (comma-separated joint values)
        joint_subs = this->create_subscription<std_msgs::msg::String>(
            "/joint_goal", 10, std::bind(&goalPubNode::jointCallback, this, std::placeholders::_1));

        
        RCLCPP_INFO(this->get_logger(), "node initialized.");
    }

private:
    bool goal_reached = false;  // Flag to track if the pose goal has been achieved

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        
     
        
        move_group_interface_.setPoseTarget(*msg);
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

        if (move_group_interface_.move() == moveit::core::MoveItErrorCode::SUCCESS)
        {
            // goal_reached=true;
            RCLCPP_INFO(this->get_logger(), "Pose goal achieved.");
            //rclcpp::shutdown();

        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Pose goal failed.");
        }
    }

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
        
        //printing current joint values
        const auto current_joint_values = move_group_interface_.getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "Current joint values: ");
        for (size_t i = 0; i < current_joint_values.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %f", i, current_joint_values[i]);
        }



        move_group_interface_.setJointValueTarget(joint_values);
        RCLCPP_INFO(this->get_logger(), "Joint goal received.");

    
        
        move_group_interface_.setPlanningTime(30.0);

        if (move_group_interface_.move() == moveit::core::MoveItErrorCode::SUCCESS)
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
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joint_constraint_subs;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<goalPubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
