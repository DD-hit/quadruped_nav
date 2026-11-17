// Coordinate and sensor-origin adaptation only; no planning or control logic.
#include <memory>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

class FrameAdapter : public rclcpp::Node {
 public:
  FrameAdapter() : Node("frame_adapter") {
    frame_ = declare_parameter<std::string>("planning_frame", "map");
    base_ = declare_parameter<std::string>("base_frame", "base_link");
    lidar_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/lio/cloud_registered");
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    buffer_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface()));
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, true);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose", rclcpp::SensorDataQoS());
    sensor_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensor_pose", rclcpp::SensorDataQoS());
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());
    odom_sub_.subscribe(this, odom_topic, rmw_qos_profile_sensor_data);
    cloud_sub_.subscribe(this, cloud_topic, rmw_qos_profile_sensor_data);
    odom_filter_ = std::make_unique<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>>(
      *buffer_, frame_, 100, get_node_logging_interface(), get_node_clock_interface(), std::chrono::seconds(1));
    cloud_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      *buffer_, frame_, 20, get_node_logging_interface(), get_node_clock_interface(), std::chrono::seconds(1));
    odom_filter_->connectInput(odom_sub_); cloud_filter_->connectInput(cloud_sub_);
    odom_filter_->registerCallback(std::bind(&FrameAdapter::odom, this, std::placeholders::_1));
    cloud_filter_->registerCallback(std::bind(&FrameAdapter::cloud, this, std::placeholders::_1));
  }
  ~FrameAdapter() override { odom_filter_.reset(); cloud_filter_.reset(); listener_.reset(); }
 private:
  void odom(const nav_msgs::msg::Odometry::ConstSharedPtr &input) {
    if (input->child_frame_id != base_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Odometry child_frame_id must be %s", base_.c_str()); return;
    }
    try {
      auto out = *input;
      const auto tr = buffer_->lookupTransform(frame_, input->header.frame_id, input->header.stamp);
      tf2::Transform outer, inner; tf2::fromMsg(tr.transform, outer); tf2::fromMsg(input->pose.pose, inner);
      tf2::toMsg(outer * inner, out.pose.pose); out.header.frame_id = frame_;
      Eigen::Matrix<double, 6, 6> rotation = Eigen::Matrix<double, 6, 6>::Zero();
      for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
        rotation(r,c)=outer.getBasis()[r][c]; rotation(r+3,c+3)=rotation(r,c);
      }
      Eigen::Matrix<double,6,6> covariance;
      for (int r=0;r<6;++r) for(int c=0;c<6;++c) covariance(r,c)=input->pose.covariance[6*r+c];
      covariance = (rotation * covariance * rotation.transpose()).eval();
      for (int r=0;r<6;++r) for(int c=0;c<6;++c) out.pose.covariance[6*r+c]=covariance(r,c);
      // Standard Odometry twist remains expressed in child/base coordinates.
      odom_pub_->publish(out);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", e.what());
    }
  }
  void cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &input) {
    try {
      auto tr = buffer_->lookupTransform(frame_, input->header.frame_id, input->header.stamp);
      auto sensor = buffer_->lookupTransform(frame_, lidar_, input->header.stamp);
      nav_msgs::msg::Odometry pose; pose.header = input->header; pose.header.frame_id=frame_; pose.child_frame_id=lidar_;
      pose.pose.pose.position.x=sensor.transform.translation.x; pose.pose.pose.position.y=sensor.transform.translation.y;
      pose.pose.pose.position.z=sensor.transform.translation.z; pose.pose.pose.orientation=sensor.transform.rotation;
      sensor_pub_->publish(pose);
      sensor_msgs::msg::PointCloud2 out; tf2::doTransform(*input, out, tr); cloud_pub_->publish(out);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", e.what());
    }
  }
  std::string frame_, base_, lidar_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>> odom_filter_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> cloud_filter_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};
int main(int argc,char**argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<FrameAdapter>()); rclcpp::shutdown(); return 0;
}
