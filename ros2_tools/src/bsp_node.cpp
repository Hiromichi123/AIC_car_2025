#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "ros2_tools/msg/lidar_pose.hpp"

class PositionController : public rclcpp::Node {
public:
  PositionController() : Node("position_controller"), Kp_linear_(0.8), Kp_angular_(1.0) {
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal", 10,
      std::bind(&PositionController::goalCallback, this, std::placeholders::_1));

    lidar_sub_ = this->create_subscription<ros2_tools::msg::LidarPose>(
      "/lidar_data", 10,
      std::bind(&PositionController::lidarCallback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mecanum_controller/reference", 10);

    RCLCPP_INFO(this->get_logger(), "Position controller node started.");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_goal_ = *msg;
    computeAndPublish();
  }

  void lidarCallback(const ros2_tools::msg::LidarPose::SharedPtr msg) {
    latest_pose_.pose.position.x = msg->x;
    latest_pose_.pose.position.y = msg->y;
    latest_pose_.pose.position.z = msg->z;
    latest_yaw_ = msg->yaw;
    computeAndPublish();
  }

  void computeAndPublish() {
    double dx = latest_goal_.pose.position.x - latest_pose_.pose.position.x;
    double dy = latest_goal_.pose.position.y - latest_pose_.pose.position.y;

    // 简单比例控制
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = this->now();
    cmd.header.frame_id = "base_link";
    cmd.twist.linear.x = Kp_linear_ * dx;
    cmd.twist.linear.y = Kp_linear_ * dy;
    double v_max = 0.5; // 限幅
    cmd.twist.linear.x = std::clamp(cmd.twist.linear.x, -v_max, v_max);
    cmd.twist.linear.y = std::clamp(cmd.twist.linear.y, -v_max, v_max);

    // 计算目标方向角
    double goal_yaw = atan2(dy, dx);
    double yaw_error = goal_yaw - latest_yaw_;
    while (yaw_error > M_PI) yaw_error -= 2*M_PI;
    while (yaw_error < -M_PI) yaw_error += 2*M_PI;

    cmd.twist.angular.z = Kp_angular_ * yaw_error;

    cmd_pub_->publish(cmd);
  }

  double Kp_linear_;
  double Kp_angular_;

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
