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

    if (!open_first_available()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开任何摄像头，请检查硬件连接");
      rclcpp::shutdown();
      return;
    }
    else {
      RCLCPP_INFO(this->get_logger(), "成功打开摄像头 /dev/camera1");
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    camera.set(cv::CAP_PROP_FPS, 30);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&GroundCameraNode::timer_callback, this));
  }

private:
  bool open_first_available() {
    camera.release();
    if (camera.open("/dev/camera1", cv::CAP_V4L2)) {
      return true;
    }
    RCLCPP_WARN(this->get_logger(), "尝试打开 /dev/camera1 失败");
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

    cv::rotate(this->frame, this->flipped_frame, cv::ROTATE_180);  // 旋转 180 度

    this->cv_image.encoding = "bgr8";
    this->cv_image.image = this->flipped_frame;
    camera_pub_->publish(*this->cv_image.toImageMsg());
  }

  cv::VideoCapture camera;

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
