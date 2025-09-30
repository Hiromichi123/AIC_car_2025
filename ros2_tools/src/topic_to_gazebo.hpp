#ifndef ROBOT_GAZEBO__GOAL_TO_CMD_VEL_HPP_
#define ROBOT_GAZEBO__GOAL_TO_CMD_VEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GoalToCmdVel : public rclcpp::Node
{
public:
  GoalToCmdVel();

private:
  // PID参数
  double linear_kp_;
  double angular_kp_;
  double max_linear_speed_;
  double max_angular_speed_;
  double goal_tolerance_;
  
  // 状态变量
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose current_goal_;
  bool reached_goal_;
  
  // 订阅者和发布者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // 回调函数
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void control_loop();
  
  // 工具函数
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat);
  double calculate_distance(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2);
  double calculate_angle_to_goal(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose);
  double normalize_angle(double angle);
};

#endif  // ROBOT_GAZEBO__GOAL_TO_CMD_VEL_HPP_