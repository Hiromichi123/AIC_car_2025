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
    PositionController()
    : Node("bsp_node"),
      Kp_linear_(declare_parameter("kp_linear", 2.0)),
      Ki_linear_(declare_parameter("ki_linear", 0.02)),
      Kd_linear_(declare_parameter("kd_linear", 0.02)),
      Kp_angular_(declare_parameter("kp_angular", 2.0)),
      Ki_angular_(declare_parameter("ki_angular", 0.02)),
      Kd_angular_(declare_parameter("kd_angular", 0.02)),
      position_threshold_(declare_parameter("position_threshold", 0.00)),
      angle_threshold_(declare_parameter("angle_threshold", 0.02)),
      rotation_first_threshold_(declare_parameter("rotation_first_threshold", 0.1))
  {
    // 目标点订阅
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal", 10, std::bind(&PositionController::goalCallback, this, std::placeholders::_1));

    // lidar数据订阅
    lidar_sub_ = this->create_subscription<ros2_tools::msg::LidarPose>(
        "/lidar_data", 10, std::bind(&PositionController::lidarCallback, this, std::placeholders::_1));

    lidar_pub = this->create_publisher<ros2_tools::msg::LidarPose>("lidar_data", 10);
    
    // 底层速度命令发布
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "bsp_node 中间层已启动，开始接收goal和lidar数据");
  }

private:
  // void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  //   latest_goal_ = *msg;
  //   has_goal_ = true;
  //   integral_dx_ = integral_dy_ = integral_yaw_ = 0.0;
  //   prev_dx_ = prev_dy_ = prev_yaw_ = 0.0;
  //   first_update_ = true;

  //   // 检查是否有目标朝向
  //   auto &q = msg->pose.orientation;
  //   if (fabs(q.x) > 0.001 || fabs(q.y) > 0.001 || fabs(q.z) > 0.001 ||
  //       fabs(q.w - 1.0) > 0.001) {
  //     // 有目标朝向
  //     target_yaw_ = extractYawFromQuaternion(q);
  //     has_target_yaw_ = true;
  //     rotating_first_ = true;
  //     // RCLCPP_INFO(this->get_logger(),
  //     //             "Goal with orientation: pos(%.2f, %.2f), yaw=%.1f°",
  //     //             msg->pose.position.x, msg->pose.position.y,
  //     //             target_yaw_ * 180.0 / M_PI);
  //   } else {
  //     // 无目标朝向
  //     has_target_yaw_ = false;
  //     rotating_first_ = false;
  //     // RCLCPP_INFO(this->get_logger(),
  //     //             "Goal without orientation: pos(%.2f, %.2f)",
  //     //             msg->pose.position.x, msg->pose.position.y);
  //   }

  //   computeAndPublish();
  // }
  rclcpp::Publisher<ros2_tools::msg::LidarPose>::SharedPtr lidar_pub;
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    ros2_tools::msg::LidarPose lidar_pose;
    lidar_pose.x = msg->pose.position.x;
    lidar_pose.y = msg->pose.position.y;
    lidar_pose.z = 0;
    lidar_pose.roll = 0;
    lidar_pose.pitch = 0;
    lidar_pose.yaw = 0;

    lidar_pub->publish(lidar_pose);
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
    rclcpp::Time now = this->get_clock()->now();
    double dt = first_update_ ? 0.0 : (now - last_time_).seconds();
    if (dt <= 0.0) {
      dt = 1e-3;
    }
    last_time_ = now;

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
          // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          //                      "Position reached, adjusting angle: %.1f° to go",
          //                      yaw_error * 180.0 / M_PI);
          return;
        }
      }

      // 完全到达
      cmd_pub_->publish(cmd);
      has_goal_ = false;
      first_update_ = true;
      //RCLCPP_INFO(this->get_logger(), "Goal fully reached!");
      return;
    }

    // 先转向再移动的逻辑
    if (has_target_yaw_ && rotating_first_) {
      double yaw_error = normalizeAngle(target_yaw_ - latest_yaw_);

      integral_yaw_ += yaw_error * dt;
      double deriv_yaw = (yaw_error - prev_yaw_) / dt;
      prev_yaw_ = yaw_error;

      if (fabs(yaw_error) > rotation_first_threshold_) {
        // 角度误差大，先原地转向
        cmd.angular.z = Kp_angular_ * yaw_error;
        cmd.angular.z += Ki_angular_ * integral_yaw_ + Kd_angular_ * deriv_yaw;
        cmd_pub_->publish(cmd);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        //                      "Rotating first: %.1f° to go",
        //                      yaw_error * 180.0 / M_PI);
        return;
      } else {
        // 转向完成，开始移动
        rotating_first_ = false;
        //RCLCPP_INFO(this->get_logger(), "Rotation complete, starting movement");
      }
    }

    // 执行平移
    double cos_yaw = cos(latest_yaw_);
    double sin_yaw = sin(latest_yaw_);
    double dx_local = dx_global * cos_yaw + dy_global * sin_yaw;
    double dy_local = -dx_global * sin_yaw + dy_global * cos_yaw;

    if (first_update_) {
      prev_dx_ = dx_local;
      prev_dy_ = dy_local;
      prev_yaw_ = normalizeAngle(target_yaw_ - latest_yaw_);
      first_update_ = false;
    }

    integral_dx_ += dx_local * dt;
    integral_dy_ += dy_local * dt;

    double deriv_dx = (dx_local - prev_dx_) / dt;
    double deriv_dy = (dy_local - prev_dy_) / dt;

    prev_dx_ = dx_local;
    prev_dy_ = dy_local;

    cmd.linear.x = Kp_linear_ * dx_local;
    cmd.linear.y = Kp_linear_ * dy_local;

    cmd.linear.x += Ki_linear_ * integral_dx_ + Kd_linear_ * deriv_dx;
    cmd.linear.y += Ki_linear_ * integral_dy_ + Kd_linear_ * deriv_dy;

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
      integral_yaw_ += yaw_error * dt;
      double deriv_yaw = (yaw_error - prev_yaw_) / dt;
      prev_yaw_ = yaw_error;

      cmd.angular.z = Kp_angular_ * yaw_error * 0.3; // 降低权重
      cmd.angular.z += Ki_angular_ * integral_yaw_ + Kd_angular_ * deriv_yaw;
    } else {
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //                      "Distance: %.2fm, vx=%.2f, vy=%.2f, ω=%.2f", distance,
    //                      cmd.linear.x, cmd.linear.y, cmd.angular.z);
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
  double Kp_linear_, Ki_linear_, Kd_linear_;
  double Kp_angular_, Ki_angular_, Kd_angular_;
  double position_threshold_;
  double angle_threshold_;
  double rotation_first_threshold_;

  double integral_dx_ = 0.0, integral_dy_ = 0.0, integral_yaw_ = 0.0;
  double prev_dx_ = 0.0, prev_dy_ = 0.0, prev_yaw_ = 0.0;
  rclcpp::Time last_time_;
  bool first_update_ = true;

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