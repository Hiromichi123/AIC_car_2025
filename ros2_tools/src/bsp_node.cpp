#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "ros2_tools/msg/lidar_pose.hpp"
#include <cmath>

class PositionController : public rclcpp::Node {
public:
  PositionController() : Node("bsp_node"), 
                         Kp_linear_(0.8), 
                         Kp_angular_(1.5),
                         has_goal_(false),
                         position_tolerance_(0.05),      // 0.05 位置容差
                         angle_tolerance_(0.1) {         // 5.7° 角度容差
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal", 10,
      std::bind(&PositionController::goalCallback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<ros2_tools::msg::LidarPose>(
      "/lidar_data", 10,
      std::bind(&PositionController::lidarCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mecanum_controller/reference", 10);

    RCLCPP_INFO(this->get_logger(), "bsp_node started.");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_goal_ = *msg;
    has_goal_ = true;
    computeAndPublish();
  }

  void lidarCallback(const ros2_tools::msg::LidarPose::SharedPtr msg) {
    latest_pose_.pose.position.x = msg->x;
    latest_pose_.pose.position.y = msg->y;
    latest_pose_.pose.position.z = msg->z;
    latest_yaw_ = msg->yaw;
    
    if (has_goal_) {
      computeAndPublish();
    }
  }

  void computeAndPublish() {
    // 全局坐标系下的位置误差
    double dx_global = latest_goal_.pose.position.x - latest_pose_.pose.position.x;
    double dy_global = latest_goal_.pose.position.y - latest_pose_.pose.position.y;
    double distance = sqrt(dx_global*dx_global + dy_global*dy_global);
    
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";

    if (distance > position_tolerance_) {
      // 全局 -> 局部坐标转换
      double cos_yaw = cos(latest_yaw_);
      double sin_yaw = sin(latest_yaw_);
      double dx_local = dx_global * cos_yaw + dy_global * sin_yaw;
      double dy_local = -dx_global * sin_yaw + dy_global * cos_yaw;

      // ✅ 只控制位置，不控制角度
      cmd.twist.linear.x = Kp_linear_ * dx_local;
      cmd.twist.linear.y = Kp_linear_ * dy_local;
      
      double v_max = 0.5;
      cmd.twist.linear.x = std::clamp(cmd.twist.linear.x, -v_max, v_max);
      cmd.twist.linear.y = std::clamp(cmd.twist.linear.y, -v_max, v_max);

      cmd.twist.angular.z = 0.0;  // ✅ 不旋转！保持当前朝向
      
      cmd_pub_->publish(cmd);
      
    } else {
      // 到达目标
      cmd_pub_->publish(cmd);  // 零速度
      has_goal_ = false;
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
    }
  }

  double Kp_linear_;
  double Kp_angular_;
  bool has_goal_;
  double position_tolerance_;  // 位置到达容差
  double angle_tolerance_;     // 角度到达容差

  geometry_msgs::msg::PoseStamped latest_goal_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  double latest_yaw_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionController>());
  rclcpp::shutdown();
  return 0;
}