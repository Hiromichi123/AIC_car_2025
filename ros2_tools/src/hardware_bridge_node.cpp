/**
 * @file hardware_bridge_node.cpp
 * @brief ROS2 Hardware Bridge Node for Real Robot Motor Control
 * 
 * This node serves as the middleware conversion layer between ROS2 cmd_vel
 * commands and the real robot's low-level motor controller protocol.
 * 
 * It handles:
 * - Serial communication with motor controller
 * - Velocity command conversion (cmd_vel -> motor commands)
 * - Encoder feedback processing (if available)
 * - Mecanum wheel kinematics for omnidirectional movement
 * 
 * Protocol format (configurable):
 * - Header: 0xAA 0x55
 * - Command type: 1 byte
 * - Payload: variable length
 * - Checksum: 1 byte (XOR of all payload bytes)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <vector>
#include <array>

// Protocol constants
constexpr uint8_t PROTOCOL_HEADER_1 = 0xAA;
constexpr uint8_t PROTOCOL_HEADER_2 = 0x55;
constexpr uint8_t CMD_VELOCITY = 0x01;
constexpr uint8_t CMD_STOP = 0x02;
constexpr uint8_t CMD_QUERY_STATUS = 0x03;

// Mecanum wheel geometry (in meters)
constexpr double WHEEL_RADIUS = 0.05;     // 50mm wheel radius
constexpr double ROBOT_LENGTH = 0.3;      // Length from center to wheel (front-back)
constexpr double ROBOT_WIDTH = 0.25;      // Width from center to wheel (left-right)

class HardwareBridgeNode : public rclcpp::Node {
public:
    HardwareBridgeNode() : Node("hardware_bridge_node"), serial_fd_(-1) {
        // Initialize last_cmd_time_ to current time to prevent undefined behavior
        last_cmd_time_ = this->now();
        
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<bool>("enable_serial", true);
        this->declare_parameter<double>("max_linear_speed", 1.0);  // m/s
        this->declare_parameter<double>("max_angular_speed", 2.0); // rad/s
        this->declare_parameter<double>("wheel_radius", WHEEL_RADIUS);
        this->declare_parameter<double>("robot_length", ROBOT_LENGTH);
        this->declare_parameter<double>("robot_width", ROBOT_WIDTH);

        // Get parameters
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        enable_serial_ = this->get_parameter("enable_serial").as_bool();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        robot_length_ = this->get_parameter("robot_length").as_double();
        robot_width_ = this->get_parameter("robot_width").as_double();

        // Initialize serial port if enabled
        if (enable_serial_) {
            if (!initSerial()) {
                RCLCPP_ERROR(this->get_logger(), 
                    "Failed to open serial port %s. Running in simulation mode.", 
                    serial_port_.c_str());
                enable_serial_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), 
                    "Serial port %s opened successfully at %d baud", 
                    serial_port_.c_str(), baud_rate_);
            }
        }

        // Create subscriber for velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&HardwareBridgeNode::cmdVelCallback, this, std::placeholders::_1));

        // Create publisher for debug/status messages
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/hardware_status", 10);

        // Create timer for sending commands at fixed rate (50Hz)
        send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&HardwareBridgeNode::sendCommandTimer, this));

        // Watchdog timer - stop motors if no cmd_vel received for 500ms
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HardwareBridgeNode::watchdogCallback, this));

        RCLCPP_INFO(this->get_logger(), "Hardware Bridge Node initialized");
        RCLCPP_INFO(this->get_logger(), "  Serial: %s", enable_serial_ ? "Enabled" : "Disabled");
        RCLCPP_INFO(this->get_logger(), "  Max linear speed: %.2f m/s", max_linear_speed_);
        RCLCPP_INFO(this->get_logger(), "  Max angular speed: %.2f rad/s", max_angular_speed_);
    }

    ~HardwareBridgeNode() {
        // Stop motors and close serial port
        if (serial_fd_ >= 0) {
            sendStopCommand();
            close(serial_fd_);
        }
    }

private:
    bool initSerial() {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            return false;
        }

        struct termios options;
        tcgetattr(serial_fd_, &options);

        // Set baud rate
        speed_t baud;
        switch (baud_rate_) {
            case 9600:   baud = B9600;   break;
            case 19200:  baud = B19200;  break;
            case 38400:  baud = B38400;  break;
            case 57600:  baud = B57600;  break;
            case 115200: baud = B115200; break;
            default:     baud = B115200; break;
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        // 8N1 mode
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag |= (CLOCAL | CREAD);

        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIOFLUSH);

        return true;
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Store latest command and update timestamp
        latest_cmd_ = *msg;
        last_cmd_time_ = this->now();
        cmd_received_ = true;  // Mark that we've received commands

        // Apply speed limits with warning if clamping occurs
        double vx = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
        double vy = std::clamp(msg->linear.y, -max_linear_speed_, max_linear_speed_);
        double omega = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        // Log warning if commands were clamped
        if (vx != msg->linear.x || vy != msg->linear.y || omega != msg->angular.z) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Velocity command clamped: vx %.2f->%.2f, vy %.2f->%.2f, omega %.2f->%.2f",
                msg->linear.x, vx, msg->linear.y, vy, msg->angular.z, omega);
        }

        // Convert to mecanum wheel velocities
        calculateWheelSpeeds(vx, vy, omega);

        RCLCPP_DEBUG(this->get_logger(), 
            "cmd_vel: vx=%.2f, vy=%.2f, omega=%.2f", vx, vy, omega);
    }

    /**
     * @brief Calculate mecanum wheel speeds from robot velocity
     * 
     * Mecanum wheel kinematics:
     * v_fl = (vx - vy - (L+W)*omega) / R
     * v_fr = (vx + vy + (L+W)*omega) / R
     * v_rl = (vx + vy - (L+W)*omega) / R
     * v_rr = (vx - vy + (L+W)*omega) / R
     * 
     * Where L = robot_length, W = robot_width, R = wheel_radius
     */
    void calculateWheelSpeeds(double vx, double vy, double omega) {
        double L_plus_W = robot_length_ + robot_width_;
        
        // Calculate wheel angular velocities (rad/s)
        wheel_speeds_[0] = (vx - vy - L_plus_W * omega) / wheel_radius_; // Front Left
        wheel_speeds_[1] = (vx + vy + L_plus_W * omega) / wheel_radius_; // Front Right
        wheel_speeds_[2] = (vx + vy - L_plus_W * omega) / wheel_radius_; // Rear Left
        wheel_speeds_[3] = (vx - vy + L_plus_W * omega) / wheel_radius_; // Rear Right
    }

    void sendCommandTimer() {
        if (!enable_serial_ || serial_fd_ < 0) {
            return;
        }

        // Build and send velocity command packet
        std::vector<uint8_t> packet;
        buildVelocityPacket(packet);
        
        ssize_t written = write(serial_fd_, packet.data(), packet.size());
        if (written < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Failed to write to serial port: error %d", errno);
        } else if (static_cast<size_t>(written) != packet.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Partial write to serial port: wrote %zd of %zu bytes", 
                written, packet.size());
        }
    }

    void buildVelocityPacket(std::vector<uint8_t>& packet) {
        packet.clear();
        packet.push_back(PROTOCOL_HEADER_1);
        packet.push_back(PROTOCOL_HEADER_2);
        packet.push_back(CMD_VELOCITY);
        
        // Send 4 wheel speeds as 16-bit signed integers (in units of 0.01 rad/s)
        uint8_t checksum = CMD_VELOCITY;
        for (int i = 0; i < 4; i++) {
            int16_t speed_int = static_cast<int16_t>(wheel_speeds_[i] * 100.0);
            uint8_t high = (speed_int >> 8) & 0xFF;
            uint8_t low = speed_int & 0xFF;
            packet.push_back(high);
            packet.push_back(low);
            checksum ^= high;
            checksum ^= low;
        }
        packet.push_back(checksum);
    }

    void sendStopCommand() {
        if (serial_fd_ < 0) return;

        std::vector<uint8_t> packet;
        packet.push_back(PROTOCOL_HEADER_1);
        packet.push_back(PROTOCOL_HEADER_2);
        packet.push_back(CMD_STOP);
        packet.push_back(CMD_STOP);  // Checksum is just the command byte

        write(serial_fd_, packet.data(), packet.size());
    }

    void watchdogCallback() {
        // Check if we haven't received commands for a while
        auto elapsed = this->now() - last_cmd_time_;
        if (elapsed.seconds() > 0.5 && cmd_received_) {
            // Stop motors
            std::fill(wheel_speeds_.begin(), wheel_speeds_.end(), 0.0);
            
            // Use throttle to prevent log spam, but still warn periodically
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "No cmd_vel received for %.1f seconds, motors stopped", 
                elapsed.seconds());
        }
    }

    // Serial communication
    int serial_fd_;
    std::string serial_port_;
    int baud_rate_;
    bool enable_serial_;

    // Robot parameters
    double max_linear_speed_;
    double max_angular_speed_;
    double wheel_radius_;
    double robot_length_;
    double robot_width_;

    // State
    geometry_msgs::msg::Twist latest_cmd_;
    rclcpp::Time last_cmd_time_;
    std::array<double, 4> wheel_speeds_ = {0.0, 0.0, 0.0, 0.0};
    bool cmd_received_ = false;  // Track if we've ever received a cmd_vel

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
