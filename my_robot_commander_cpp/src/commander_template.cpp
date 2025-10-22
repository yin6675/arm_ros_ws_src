#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>                  //为open_gripper话题服务 只需传入true或false
#include <example_interfaces/msg/float64_multi_array.hpp>   //为joints话题服务 需要传入各个关节的值 但还是建议自己创建一个单独的interfaces来服务joint话题
#include <my_robot_interfaces/msg/pose_command.hpp>          //为pose_command话题服务 需要传入位置和姿态信息


using MoveGroupInterface =  moveit::planning_interface::MoveGroupInterface;   //定义一个变量类型，方便后续使用
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

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

        open_gripper_sub_ = node_->create_subscription<Bool>("open_gripper", 10, std::bind(&Commander::openGripperCallback, this, _1));   //创造一个订阅者，订阅“open_gripper话题”

        joints_cmd_sub_ = node_->create_subscription<FloatArray>("joint_command", 10, std::bind(&Commander::jointCmdCallback, this, _1));   //创造一个订阅者，订阅“joint_command话题”

        pose_cmd_sub_ = node_->create_subscription<PoseCmd>("pose_command", 10, std::bind(&Commander::poseCmdCallback, this, _1));   //创造一个订阅者，订阅“jpose_command话题”
    }

    void goToNameTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();   //设置初始位置
        arm_->setNamedTarget(name);    //设置目标位置
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double>& joints)   //关节转动的角度，一般要调用好几次来控制机械臂的各个关节
    {
        arm_->setStartStateToCurrentState();  //设置初始位置
        arm_->setJointValueTarget(joints);   //设置目标位置
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x, double y, double z, double roll, double pitch, double yaw, bool cartesian_path = false)   // 建立三维坐标，输入坐标使得机械臂运动到该位置
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
        //若不是卡笛尔路径，则直接设置目标位置
        if (!cartesian_path){
            arm_->setPoseTarget(target_pose);   //设置目标位置
            planAndExecute(arm_);
        }
        //若是卡笛尔路径，则设置路径
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints, 0.01, trajectory);

            if (fraction == 1)
                {
                     arm_->execute(trajectory);
                }
        }

    }

    void openGripper()   //打开夹爪的函数
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");
        planAndExecute(gripper_);
    }

    void closeGripper()   //关闭夹爪的函数
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_close");
        planAndExecute(gripper_);
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

    void  openGripperCallback(const Bool &msg)   //创建一个回调函数，用于订阅“open_gripper话题”
    {

        if(msg.data)   //若数据为真，则打开夹爪；否则关闭夹爪
        {
            openGripper();
        }
        else
        {
            closeGripper();
        }
    }


    void jointCmdCallback(const FloatArray &msg)   //创建一个回调函数，用于订阅“joints_cmd”话题  基本不用
    {

        auto joints = msg.data;   //获取话题中的数据，并存入joints变量中
        if(joints.size() == 6)   //判断数据的大小是否为6（机械臂有6个关节）
        {
            goToJointTarget(joints);  //调用goToJointTarget函数，使机械臂运动到指定关节位置
        }
        

    }

    void poseCmdCallback(const PoseCmd &msg)   //创建一个回调函数，用于订阅“pose_command”话题
    {
        goToPoseTarget(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw, msg.cartesian_path);   //调用goToPoseTarget函数，使机械臂运动到指定位置
    }


    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;      //创建一个订阅者对象，用于订阅“open_gripper”话题
    rclcpp::Subscription<FloatArray>::SharedPtr joints_cmd_sub_;  //创建一个订阅者对象，用于订阅“joints_cmd”话题
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;   //创建一个订阅者对象，用于订阅“pose_cmd”话题

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

//最后的最后要记得创建一个可执行文件的CMakeLists.txt文件

