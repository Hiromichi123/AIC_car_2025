#include "base.h"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

RobotBase::RobotBase() :
        Node("odometry_publisher"),
        linear_velocity_x_(0.0),
        linear_velocity_y_(0.0),
        angular_velocity_z_(0.0),
        last_vel_time_(this->now()),
        vel_dt_(0.0),
        x_pos_(0.0),
        y_pos_(0.0),
        heading_(0.0) {
    
    // Declare and get parameters
    this->declare_parameter("linear_scale_x", 1.0);
    this->declare_parameter("linear_scale_y", 1.0);
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_footprint_frame", "base_footprint");
    
    this->get_parameter("linear_scale_x", linear_scale_x);
    this->get_parameter("linear_scale_y", linear_scale_y);
    this->get_parameter("odom_frame", odom_frame);
    this->get_parameter("base_footprint_frame", base_footprint_frame);
    
    // Create publisher and subscriber
    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/sub_vel", 50, std::bind(&RobotBase::velCallback, this, std::placeholders::_1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/pub_odom", 50);
    
    // Create TF broadcaster
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void RobotBase::velCallback(const geometry_msgs::msg::Twist::SharedPtr twist) {
    rclcpp::Time current_time = this->now();
    linear_velocity_x_ = twist->linear.x * linear_scale_x;
    linear_velocity_y_ = twist->linear.y * linear_scale_y;
    angular_velocity_z_ = twist->angular.z;
    vel_dt_ = (current_time - last_vel_time_).seconds();
    last_vel_time_ = current_time;
    
    //compute odometry in a typical way given the velocities of the robot
    double delta_heading = angular_velocity_z_ * vel_dt_; //radians
    double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; //m
    double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; //m
    
    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    
    //calculate robot's heading in quaternion angle
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, heading_);
    
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_footprint_frame;
    
    // robot's position in x,y and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    
    // robot's heading in quaternion
    odom.pose.pose.orientation = tf2::toMsg(odom_quat);
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    
    // linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x_;
    odom.twist.twist.linear.y = linear_velocity_y_;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    
    // angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z_;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
    
    odom_publisher_->publish(odom);
}
