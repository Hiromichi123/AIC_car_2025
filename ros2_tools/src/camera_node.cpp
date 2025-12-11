#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

class GroundCameraNode : public rclcpp::Node {
public:
  GroundCameraNode() : Node("camera_node") {
    camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/video", 1);

    preferred_device_ = this->declare_parameter<std::string>("camera_device", "/dev/video0");
    fallback_devices_ = {
      preferred_device_,
      "/dev/video0",
      "/dev/video1",
      "/dev/camera0",
      "/dev/camera1"
    };

    if (!open_first_available()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开任何摄像头，请检查硬件连接");
      rclcpp::shutdown();
      return;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    camera.set(cv::CAP_PROP_FPS, 30);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&GroundCameraNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Camera node started，设备: %s", active_device_.c_str());
  }

private:
  bool open_first_available() {
    for (const auto &device : fallback_devices_) {
      if (device.empty()) {
        continue;
      }
      camera.release();
      if (camera.open(device, cv::CAP_V4L2)) {
        active_device_ = device;
        return true;
      }
      RCLCPP_WARN(this->get_logger(), "尝试打开 %s 失败", device.c_str());
    }
    active_device_.clear();
    return false;
  }

  bool ensure_camera_ready() {
    if (camera.isOpened()) {
      return true;
    }
    RCLCPP_WARN(this->get_logger(), "摄像头连接丢失，尝试重新打开...");
    return open_first_available();
  }

  void timer_callback() {
    if (!ensure_camera_ready()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "仍然无法连接摄像头");
      return;
    }

    camera >> this->frame;

    if (this->frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "camera空帧，跳过本次发布");
      return;
    }

    cv::flip(this->frame, this->flipped_frame, 0);  // 0 表示上下翻转

    this->cv_image.encoding = "bgr8";
    this->cv_image.image = this->flipped_frame;
    camera_pub_->publish(*this->cv_image.toImageMsg());
  }

  cv::VideoCapture camera;
  std::string preferred_device_;
  std::vector<std::string> fallback_devices_;
  std::string active_device_;

  cv::Mat frame;
  cv::Mat flipped_frame;
  cv_bridge::CvImage cv_image;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundCameraNode>());
  rclcpp::shutdown();
  return 0;
}
