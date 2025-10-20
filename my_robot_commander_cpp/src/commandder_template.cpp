#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using MoveGroupInterface =  moveit::planning_interface::MoveGroupInterface;

class Commander
{

public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        // Initialize the MoveGroupInterface
        node_= node;
        arm_  = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    }

    void goToNameTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();   //设置初始位置
        arm_->setNamedTarget(name);    //设置目标位置
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();  //设置初始位置
        arm_->setJointValueTarget(joints);   //设置目标位置
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();  //设置初始位置

        if (!cartesian_path){
            arm_->setPoseTarget(target_pose);   //设置目标位置
            planAndExecute(arm_);
        }
        else{
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);
        }

    }

private:

    void planAndExecute(const std::shared_ptr<MoveGroupInterface>& interface)   //创建一个函数，用于规划并执行运动
    {

        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            interface->execute(plan);
        }

    }

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commandder");
    auto conmander = Commander(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}