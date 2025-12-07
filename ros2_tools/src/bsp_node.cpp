/*
 * 功能:
 * 1.订阅目标点（goal）和 雷达数据（lidar_data）
 * 2.根据当前位置和目标点计算速度指令（cmd_vel）
 * 3.发布速度指令给底层控制节点
 */
#include "ros2_tools/msg/lidar_pose.hpp"
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class PositionController : public rclcpp::Node {
public:
  PositionController() : Node("bsp_node"),
        Kp_linear_(1.5), Kp_angular_(2.0),
        position_threshold_(0.05),     // 5cm 认为到达
        angle_threshold_(0.05),        // ~3° 认为朝向正确
        rotation_first_threshold_(0.2) // 角度误差 > 11° 时先转向
  {
    // 目标点订阅
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal", 10, std::bind(&PositionController::goalCallback, this, std::placeholders::_1));

    // lidar数据订阅
    lidar_sub_ = this->create_subscription<ros2_tools::msg::LidarPose>(
        "/lidar_data", 10, std::bind(&PositionController::lidarCallback, this, std::placeholders::_1));

    // 底层速度命令发布
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "bsp_node 中间层已启动，开始接收goal和lidar数据");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    latest_goal_ = *msg;
    has_goal_ = true;

    // 检查是否有目标朝向
    auto &q = msg->pose.orientation;
    if (fabs(q.x) > 0.001 || fabs(q.y) > 0.001 || fabs(q.z) > 0.001 ||
        fabs(q.w - 1.0) > 0.001) {
      // 有目标朝向
      target_yaw_ = extractYawFromQuaternion(q);
      has_target_yaw_ = true;
      rotating_first_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Goal with orientation: pos(%.2f, %.2f), yaw=%.1f°",
                  msg->pose.position.x, msg->pose.position.y,
                  target_yaw_ * 180.0 / M_PI);
    } else {
      // 无目标朝向
      has_target_yaw_ = false;
      rotating_first_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "Goal without orientation: pos(%.2f, %.2f)",
                  msg->pose.position.x, msg->pose.position.y);
    }

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

  // 计算并发布cmd_vel
  void computeAndPublish() {
    double dx_global = latest_goal_.pose.position.x - latest_pose_.pose.position.x;
    double dy_global = latest_goal_.pose.position.y - latest_pose_.pose.position.y;
    double distance = sqrt(dx_global * dx_global + dy_global * dy_global);
    geometry_msgs::msg::Twist cmd;

    // 检查是否到达位置
    if (distance < position_threshold_) {
      // 位置已到达
      if (has_target_yaw_) {
        double yaw_error = normalizeAngle(target_yaw_ - latest_yaw_);

        if (fabs(yaw_error) > angle_threshold_) {
          // 需要调整角度
          cmd.angular.z = Kp_angular_ * yaw_error;
          cmd_pub_->publish(cmd);
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                               "Position reached, adjusting angle: %.1f° to go",
                               yaw_error * 180.0 / M_PI);
          return;
        }
      }

      // 完全到达
      cmd_pub_->publish(cmd);
      has_goal_ = false;
      RCLCPP_INFO(this->get_logger(), "Goal fully reached!");
      return;
    }

    // 先转向再移动的逻辑
    if (has_target_yaw_ && rotating_first_) {
      double yaw_error = normalizeAngle(target_yaw_ - latest_yaw_);

      if (fabs(yaw_error) > rotation_first_threshold_) {
        // 角度误差大，先原地转向
        cmd.angular.z = Kp_angular_ * yaw_error;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Rotating first: %.1f° to go",
                             yaw_error * 180.0 / M_PI);
        return;
      } else {
        // 转向完成，开始移动
        rotating_first_ = false;
        RCLCPP_INFO(this->get_logger(), "Rotation complete, starting movement");
      }
    }

    // 执行平移
    double cos_yaw = cos(latest_yaw_);
    double sin_yaw = sin(latest_yaw_);
    double dx_local = dx_global * cos_yaw + dy_global * sin_yaw;
    double dy_local = -dx_global * sin_yaw + dy_global * cos_yaw;

    cmd.linear.x = Kp_linear_ * dx_local;
    cmd.linear.y = Kp_linear_ * dy_local;

    // 速度限幅（保持方向）
    double v_max = 0.5;
    double v_magnitude =
        sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y);
    if (v_magnitude > v_max) {
      double scale = v_max / v_magnitude;
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
    }

    // 移动时微调角度（如果有目标朝向）
    if (has_target_yaw_ && !rotating_first_) {
      double yaw_error = normalizeAngle(target_yaw_ - latest_yaw_);
      cmd.angular.z = Kp_angular_ * yaw_error * 0.3; // 降低权重
    } else {
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Distance: %.2fm, vx=%.2f, vy=%.2f, ω=%.2f", distance,
                         cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  // 角度归一化
  double normalizeAngle(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  // 从 Quaternion 提取 yaw
  double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return atan2(siny_cosp, cosy_cosp);
  }

  // 常规参数
  double Kp_linear_, Kp_angular_;
  double position_threshold_;
  double angle_threshold_;
  double rotation_first_threshold_;

  // 状态
  bool has_goal_ = false;
  bool has_target_yaw_ = false;
  bool rotating_first_ = false; // 标记是否在"先转向"阶段

  // 目标点信息
  geometry_msgs::msg::PoseStamped latest_goal_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  double latest_yaw_ = 0.0;
  double target_yaw_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ros2_tools::msg::LidarPose>::SharedPtr lidar_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionController>());
  rclcpp::shutdown();
  return 0;
}