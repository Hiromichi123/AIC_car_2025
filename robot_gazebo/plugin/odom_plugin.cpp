#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace gazebo
{
class PublishAbsolutePose : public ModelPlugin
{
public:
  PublishAbsolutePose() : ModelPlugin() {}

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model = _model;

    // 初始化 ROS 2 节点
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    this->ros_node = rclcpp::Node::make_shared("odom_plugin_node");

    // 获取参数：话题名
    if (_sdf->HasElement("topic_name"))
      topic_name = _sdf->Get<std::string>("topic_name");
    else
      topic_name = "/absolute_pose";

    // 获取参数：发布频率
    if (_sdf->HasElement("publish_rate"))
      publish_rate = _sdf->Get<double>("publish_rate");
    else
      publish_rate = 50.0;  // 默认50Hz

    // 创建 Publisher
    odom_pub = ros_node->create_publisher<nav_msgs::msg::Odometry>(topic_name, 10);

    // 创建更新事件连接
    update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PublishAbsolutePose::OnUpdate, this));

    last_pub_time = model->GetWorld()->SimTime();
    RCLCPP_INFO(ros_node->get_logger(), "PublishAbsolutePose plugin loaded, publishing on: %s", topic_name.c_str());
  }

private:
  void OnUpdate()
  {
    // 控制发布频率
    common::Time current_time = model->GetWorld()->SimTime();
    double dt = (current_time - last_pub_time).Double();
    if (dt < (1.0 / publish_rate))
      return;

    last_pub_time = current_time;

    // 获取模型全局位姿
    ignition::math::Pose3d pose = model->WorldPose();

    // 封装 Odometry 消息
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp.sec = current_time.sec;
    odom_msg.header.stamp.nanosec = current_time.nsec;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = model->GetName();

    odom_msg.pose.pose.position.x = pose.Pos().X();
    odom_msg.pose.pose.position.y = pose.Pos().Y();
    odom_msg.pose.pose.position.z = pose.Pos().Z();

    odom_msg.pose.pose.orientation.x = pose.Rot().X();
    odom_msg.pose.pose.orientation.y = pose.Rot().Y();
    odom_msg.pose.pose.orientation.z = pose.Rot().Z();
    odom_msg.pose.pose.orientation.w = pose.Rot().W();

    // 发布
    odom_pub->publish(odom_msg);
  }

private:
  physics::ModelPtr model;
  event::ConnectionPtr update_connection;

  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  std::string topic_name;
  double publish_rate;
  common::Time last_pub_time;
};

// 注册插件
GZ_REGISTER_MODEL_PLUGIN(PublishAbsolutePose)
}  // namespace gazebo
