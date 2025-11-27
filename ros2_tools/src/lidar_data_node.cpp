#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros2_tools/msg/lidar_pose.hpp"

class LidarDataNode : public rclcpp::Node {
public:
    LidarDataNode() : Node("lidar_data_node") {
        // Declare parameters for mode selection
        this->declare_parameter<bool>("use_simulation", true);
        this->declare_parameter<std::string>("simulation_odom_topic", "/absolute_pose");
        this->declare_parameter<std::string>("real_robot_odom_topic", "/aft_mapped_to_init");
        
        // Get parameters
        using_gazebo_ = this->get_parameter("use_simulation").as_bool();
        std::string sim_topic = this->get_parameter("simulation_odom_topic").as_string();
        std::string real_topic = this->get_parameter("real_robot_odom_topic").as_string();
        
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        // LidarPose publisher
        lidar_pub = this->create_publisher<ros2_tools::msg::LidarPose>("lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "lidar_data publisher created");

        // Subscribe to PointLIO odometry (real robot mode)
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            real_topic, 10, 
            std::bind(&LidarDataNode::odomCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Real robot odometry subscription: %s", real_topic.c_str());

        // Subscribe to Gazebo odometry (simulation mode)
        local_position_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            sim_topic, 10, 
            std::bind(&LidarDataNode::localPositionCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Simulation odometry subscription: %s", sim_topic.c_str());
        
        RCLCPP_INFO(this->get_logger(), "LidarDataNode initialized in %s mode", 
            using_gazebo_ ? "SIMULATION" : "REAL ROBOT");
    }

    // Real robot lidar odometry callback
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!using_gazebo_)
            msgDispose(msg->pose.pose); // Real robot mode - process lidar data
    }

    // Simulation odometry callback
    void localPositionCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (using_gazebo_)
            msgDispose(msg->pose.pose); // Simulation mode - process wheel odometry
    }

    // Unified message processing function
    void msgDispose(const geometry_msgs::msg::Pose &pose) {
        // Convert quaternion to Euler angles
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // Normalize angles to positive range
        if (roll < 0) roll += 2 * M_PI;
        if (pitch < 0) pitch += 2 * M_PI;
        if (yaw < 0) yaw += 2 * M_PI;

        // Fill LidarPose message
        lidar_pose.x = x;
        lidar_pose.y = y;
        lidar_pose.z = z;
        lidar_pose.roll = roll;
        lidar_pose.pitch = pitch;
        lidar_pose.yaw = yaw;

        lidar_pub->publish(lidar_pose);

        static int counter = 0;
        if (++counter >= 50) {
            RCLCPP_INFO(
                this->get_logger(),
                "Position=(%.2f, %.2f, %.2f), Orientation=(%.2f, %.2f, %.2f) rad", 
                x, y, z, roll, pitch, yaw);
                counter = 0;
        }
    }

private:
    bool using_gazebo_;
    
    rclcpp::Publisher<ros2_tools::msg::LidarPose>::SharedPtr lidar_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub;

    ros2_tools::msg::LidarPose lidar_pose;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarDataNode>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar_data_node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}