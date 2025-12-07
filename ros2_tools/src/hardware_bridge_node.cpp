/*
 * 功能:
 * - 串口通信与电机控制器
 * - 速度命令转换 (cmd_vel -> 底盘协议)
 * - 编码器反馈处理 (暂无)
 *
 * 与 Python 版底盘驱动保持一致的协议:
 * - 固定帧头: 0xAA 0xBB 0x0A 0x12 0x02
 * - 负载: Vx、Vy、Omega（三个 16 位有符号整数，小端，单位 0.001）
 * - 结束字节: 0x00
 * - 启动握手: 上电后发送 0x11 + 9 个 0x00
 * - 速度命令频率: 50Hz
 * - 看门狗: 500ms 内无新命令则停止电机
 * - 默认串口: /dev/ttyCH341USB0, 波特率 115200
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

// 默认串口
constexpr char DEFAULT_SERIAL_PORT[] = "/dev/ttyCH341USB0";
constexpr int DEFAULT_BAUD_RATE = 115200;

// 协议定义
constexpr uint8_t PROTOCOL_HEADER_1 = 0xAA;
constexpr uint8_t PROTOCOL_HEADER_2 = 0xBB;
constexpr uint8_t PROTOCOL_CMD_BYTE_1 = 0x0A;
constexpr uint8_t PROTOCOL_CMD_BYTE_2 = 0x12;
constexpr uint8_t PROTOCOL_CMD_BYTE_3 = 0x02;
constexpr uint8_t PROTOCOL_TAIL = 0x00;
constexpr double VELOCITY_SCALE = 1000.0;
constexpr std::array<uint8_t, 10> STARTUP_SEQUENCE = {0x11, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00};

// 默认速度限制
constexpr double MAX_LINEAR_SPEED = 0.5;   // m/s
constexpr double MAX_ANGULAR_SPEED = 1.0;  // rad/s

class HardwareBridgeNode : public rclcpp::Node {
public:
    HardwareBridgeNode() : Node("hardware_bridge_node"), serial_fd_(-1) {
        // 初始化 last_cmd_time_ 为当前时间，防止ub
        last_cmd_time_ = this->now();
        
        // 声明参数
        this->declare_parameter<std::string>("serial_port", DEFAULT_SERIAL_PORT);
        this->declare_parameter<int>("baud_rate", DEFAULT_BAUD_RATE);
        this->declare_parameter<double>("max_linear_speed", MAX_LINEAR_SPEED);
        this->declare_parameter<double>("max_angular_speed", MAX_ANGULAR_SPEED);
        // 获取参数
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();

        // 初始化串口
        enable_serial_ = true;
        if (enable_serial_) {
            if (!initSerial()) {
                RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s，运行仿真模式。", serial_port_.c_str());
                enable_serial_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "串口 %s 成功打开，波特率 %d", serial_port_.c_str(), baud_rate_);
            }
        }

        // 从bsp订阅速度
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&HardwareBridgeNode::cmdVelCallback, this, std::placeholders::_1));

        // 发布状态上下文
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/hardware_status", 10);

        // cmd定时器 50hz
        send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&HardwareBridgeNode::sendCommandTimer, this));

        // 看门狗定时器 - 如果500ms内未收到cmd_vel，则停止电机
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&HardwareBridgeNode::watchdogCallback, this));

        RCLCPP_INFO(this->get_logger(), "硬件桥接节点已启动");
        RCLCPP_INFO(this->get_logger(), "串口: %s", enable_serial_ ? "已启用" : "已禁用");
        RCLCPP_INFO(this->get_logger(), "当前最大线速度: %.2f m/s", max_linear_speed_);
        RCLCPP_INFO(this->get_logger(), "当前最大角速度: %.2f rad/s", max_angular_speed_);
    }

    ~HardwareBridgeNode() {
        // 析构串口
        if (serial_fd_ >= 0) {
            sendStopCommand();
            close(serial_fd_);
        }
    }

private:
    // 初始化串口
    bool initSerial() {
        // O_RDWR 读写模式 | O_NOCTTY 不成为控制终端 | O_NDELAY 非阻塞
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            return false;
        }

        // 获取当前串口设置
        struct termios options;
        tcgetattr(serial_fd_, &options);

        // 默认br115200
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

        // 8N1（常见模式：8数据位，无校验，1停止位）
        options.c_cflag &= ~PARENB; // 无校验
        options.c_cflag &= ~CSTOPB; // 1停止位
        options.c_cflag &= ~CSIZE; // 清除数据位设置
        options.c_cflag |= CS8; // 8数据位
        options.c_cflag |= (CLOCAL | CREAD); // CLOCAL让程序不受modem信号影响，CREAD启用接收

        // Raw mode
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        tcsetattr(serial_fd_, TCSANOW, &options);
        tcflush(serial_fd_, TCIOFLUSH);

        sendStartupSequence();

        return true;
    }

    // cmd_vel回调
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->now();
        cmd_received_ = true;

        // 限速 线速度度和角速度
        double vx = std::clamp(msg->linear.x, -max_linear_speed_, max_linear_speed_);
        double vy = std::clamp(msg->linear.y, -max_linear_speed_, max_linear_speed_);
        double omega = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        // 到达限速警告
        if (vx != msg->linear.x || vy != msg->linear.y || omega != msg->angular.z) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "到达限速: vx %.2f->%.2f, vy %.2f->%.2f, omega %.2f->%.2f",
                msg->linear.x, vx, msg->linear.y, vy, msg->angular.z, omega);
        }

        current_vx_ = vx;
        current_vy_ = vy;
        current_omega_ = omega;
    }

    // 定时发送速度命令
    void sendCommandTimer() {
        if (!enable_serial_ || serial_fd_ < 0) return;

        // 构建并发送速度命令包
        std::vector<uint8_t> packet;
        buildVelocityPacket(packet);
        
        ssize_t written = write(serial_fd_, packet.data(), packet.size());
        if (written < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "写入串口失败，错误码 %d", errno);
        } else if (static_cast<size_t>(written) != packet.size()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "写入串口不完整：写入了 %zd 字节，共 %zu 字节", written, packet.size());
        }
    }

    // 构建速度命令包
    void buildVelocityPacket(std::vector<uint8_t>& packet) {
        packet.clear();
        packet.push_back(PROTOCOL_HEADER_1);
        packet.push_back(PROTOCOL_HEADER_2);
        packet.push_back(PROTOCOL_CMD_BYTE_1);
        packet.push_back(PROTOCOL_CMD_BYTE_2);
        packet.push_back(PROTOCOL_CMD_BYTE_3);

        appendInt16LE(packet, toProtocolValue(current_vx_));
        appendInt16LE(packet, toProtocolValue(current_vy_));
        appendInt16LE(packet, toProtocolValue(current_omega_));

        packet.push_back(PROTOCOL_TAIL);
    }

    // 发送停止命令
    void sendStopCommand() {
        if (serial_fd_ < 0) return;

        double prev_vx = current_vx_;
        double prev_vy = current_vy_;
        double prev_omega = current_omega_;

        current_vx_ = 0.0;
        current_vy_ = 0.0;
        current_omega_ = 0.0;

        std::vector<uint8_t> packet;
        buildVelocityPacket(packet);
        write(serial_fd_, packet.data(), packet.size());

        current_vx_ = prev_vx;
        current_vy_ = prev_vy;
        current_omega_ = prev_omega;
    }

    // 检查是否长时间未收到命令（2hz）
    void watchdogCallback() {
        auto elapsed = this->now() - last_cmd_time_;
        if (elapsed.seconds() > 0.5 && cmd_received_) {
            current_vx_ = 0.0;
            current_vy_ = 0.0;
            current_omega_ = 0.0;
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "长时间未收到 cmd_vel（%.1f 秒），电机已停止", elapsed.seconds());
        }
    }

    // 转换为协议值
    int16_t toProtocolValue(double value) const {
        double scaled = std::round(value * VELOCITY_SCALE);
        if (scaled > 32767.0) scaled = 32767.0;
        if (scaled < -32768.0) scaled = -32768.0;
        return static_cast<int16_t>(scaled);
    }

    // 以小端格式追加16位整数到数据包
    void appendInt16LE(std::vector<uint8_t>& packet, int16_t value) const {
        packet.push_back(static_cast<uint8_t>(value & 0xFF));
        packet.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    }

    // 发送启动序列
    void sendStartupSequence() {
        if (serial_fd_ < 0) return;
        ssize_t written = write(serial_fd_, STARTUP_SEQUENCE.data(), STARTUP_SEQUENCE.size());
        if (written != static_cast<ssize_t>(STARTUP_SEQUENCE.size())) {
            RCLCPP_WARN(this->get_logger(), "写入启动序列不完整：写入了 %zd 字节，共 %zu 字节", written, STARTUP_SEQUENCE.size());
        }
        usleep(1000);  // 给控制器 1ms 处理时间
    }

    // 串口
    int serial_fd_;
    std::string serial_port_;
    int baud_rate_;
    bool enable_serial_;

    // 机器人参数
    double max_linear_speed_;
    double max_angular_speed_;
    double current_vx_ = 0.0;
    double current_vy_ = 0.0;
    double current_omega_ = 0.0;

    // 状态
    rclcpp::Time last_cmd_time_;
    bool cmd_received_ = false;  // 记录是否曾接收到 cmd_vel
    
    // ros组
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
