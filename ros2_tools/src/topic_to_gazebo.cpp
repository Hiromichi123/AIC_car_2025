#include "robot_gazebo/goal_to_cmd_vel.hpp"
#include <cmath>

GoalToCmdVel::GoalToCmdVel() : Node("goal_to_cmd_vel") {
  this->declare_parameter<double>("linear_kp", 0.5);
  this->declare_parameter<double>("angular_kp", 1.0);
  this->declare_parameter<double>("max_linear_speed", 0.5);
  this->declare_parameter<double>("max_angular_speed", 1.0);
  this->declare_parameter<double>("goal_tolerance", 0.1);
  
  linear_kp_ = this->get_parameter("linear_kp").as_double();
  angular_kp_ = this->get_parameter("angular_kp").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  
  reached_goal_ = true;
  
  // 创建订阅者
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/diff_drive_base_controller/odom", 10,
    std::bind(&GoalToCmdVel::odom_callback, this, std::placeholders::_1));
    
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal", 10,
    std::bind(&GoalToCmdVel::goal_callback, this, std::placeholders::_1));
    
  // 创建发布者
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/diff_drive_base_controller/cmd_vel", 10);
    
  // 创建控制定时器 (10Hz)
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&GoalToCmdVel::control_loop, this));
    
  RCLCPP_INFO(this->get_logger(), "目标追踪节点已启动");
}

void GoalToCmdVel::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  
  // 调试信息
  double x = current_pose_.position.x;
  double y = current_pose_.position.y;
  RCLCPP_DEBUG(this->get_logger(), "当前位置: x=%.2f, y=%.2f", x, y);
}

void GoalToCmdVel::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_goal_ = msg->pose;
  reached_goal_ = false;
  
  double x = current_goal_.position.x;
  double y = current_goal_.position.y;
  RCLCPP_INFO(this->get_logger(), "收到新目标: x=%.2f, y=%.2f", x, y);
}

double GoalToCmdVel::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat)
{
  tf2::Quaternion q(
    quat.x,
    quat.y,
    quat.z,
    quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double GoalToCmdVel::calculate_distance(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double GoalToCmdVel::calculate_angle_to_goal(const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & goal_pose)
{
  double dx = goal_pose.position.x - current_pose.position.x;
  double dy = goal_pose.position.y - current_pose.position.y;
  return std::atan2(dy, dx);
}

double GoalToCmdVel::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void GoalToCmdVel::control_loop()
{
  // 检查是否有有效的位置和目标
  if (reached_goal_) {
    return;
  }
  
  // 计算到目标的距离
  double distance = calculate_distance(current_pose_, current_goal_);
  
  // 检查是否到达目标
  if (distance < goal_tolerance_) {
    if (!reached_goal_) {
      RCLCPP_INFO(this->get_logger(), "已到达目标位置！");
      reached_goal_ = true;
    }
    
    // 发布零速度停止
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);
    return;
  }
  
  // 获取当前朝向
  double current_yaw = get_yaw_from_quaternion(current_pose_.orientation);
  
  // 计算目标方向角度
  double target_yaw = calculate_angle_to_goal(current_pose_, current_goal_);
  
  // 计算角度误差（归一化到[-pi, pi]）
  double angle_error = normalize_angle(target_yaw - current_yaw);
  
  // PID控制
  double linear_speed = linear_kp_ * distance;
  double angular_speed = angular_kp_ * angle_error;
  
  // 速度限制
  linear_speed = std::max(std::min(linear_speed, max_linear_speed_), 0.0);
  angular_speed = std::max(std::min(angular_speed, max_angular_speed_), -max_angular_speed_);
  
  // 当角度误差较大时，优先旋转
  if (std::abs(angle_error) > M_PI / 4.0) {  // 45度
    linear_speed = 0.0;
  }
  
  // 创建并发布控制命令
  geometry_msgs::msg::Twist cmd_vel_msg;
  cmd_vel_msg.linear.x = linear_speed;
  cmd_vel_msg.angular.z = angular_speed;
  
  cmd_vel_pub_->publish(cmd_vel_msg);
  
  // 调试信息
  RCLCPP_DEBUG(
    this->get_logger(),
    "距离: %.2fm, 角度误差: %.1f°, 速度: lin=%.2f, ang=%.2f",
    distance, angle_error * 180.0 / M_PI, linear_speed, angular_speed);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalToCmdVel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}