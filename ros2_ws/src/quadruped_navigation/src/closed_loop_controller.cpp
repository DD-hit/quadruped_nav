#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadruped_interfaces/msg/navigation_state.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quadruped_interfaces/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include "bspline_opt/uniform_bspline.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("trajectory_controller")
  {
    state_pub_ = create_publisher<quadruped_interfaces::msg::NavigationState>("planning/controller_state", rclcpp::QoS(1).reliable().transient_local());
    task_sub_ = create_subscription<std_msgs::msg::String>("planning/task", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        task_id_ = msg->data; task_started_ = now(); receive_traj_ = false; have_goal_ = false; obstacle_stop_=false; planner_executing_=false; recovery_trajectory_=false;
        publishStop(); publishExecutionFrozen(false); publishState("IDLE");
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("planning/goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (task_id_.empty() || rclcpp::Time(msg->header.stamp) < task_started_) return;
        goal_yaw_ = tf2::getYaw(msg->pose.orientation);
        goal_position_ = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y); have_goal_ = true;
      });
    planner_state_sub_=create_subscription<quadruped_interfaces::msg::NavigationState>(
      "planning/planner_state",rclcpp::QoS(10).reliable().transient_local(),
      [this](quadruped_interfaces::msg::NavigationState::ConstSharedPtr m) {
        if(task_id_.empty() || m->task_id!=task_id_ || rclcpp::Time(m->stamp)<task_started_) return;
        if(m->state=="EMERGENCY_STOP") {
          obstacle_stop_=true;emergency_stamp_=m->stamp;planner_executing_=false;
          recovery_trajectory_=receive_traj_ && !stationary_trajectory_ && trajectory_stamp_>=emergency_stamp_;
          publishStop();
        } else {
          planner_executing_=m->state=="EXEC_TRAJ";
          if(obstacle_stop_ && planner_executing_ && recovery_trajectory_) obstacle_stop_=false;
        }
      });
    finish_yaw_ = declare_parameter<double>("finish_yaw", 0.15);
    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.5);
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);

    bspline_sub_ = create_subscription<quadruped_interfaces::msg::Bspline>(
        "planning/bspline", 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>("planning/execution_frozen", 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    last_update_time_ = now();
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  std::string task_id_;
  bool obstacle_stop_{false}, planner_executing_{false}, recovery_trajectory_{false}, stationary_trajectory_{false};
  rclcpp::Time emergency_stamp_{0,0,RCL_ROS_TIME},trajectory_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<quadruped_interfaces::msg::NavigationState>::SharedPtr planner_state_sub_;
  rclcpp::Time task_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  Eigen::Vector2d goal_position_{Eigen::Vector2d::Zero()};
  bool have_goal_{false}; double goal_yaw_{0.0}, finish_yaw_{0.15}, odom_timeout_{0.5};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<quadruped_interfaces::msg::NavigationState>::SharedPtr state_pub_;
  void publishState(const std::string &state) {
    quadruped_interfaces::msg::NavigationState msg;
    msg.stamp = now(); msg.task_id = task_id_; msg.state = state; state_pub_->publish(msg);
  }
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    cmd_vel_pub_->publish(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const quadruped_interfaces::msg::Bspline::ConstSharedPtr msg)
  {
    if (task_id_.empty() || rclcpp::Time(msg->start_time) < task_started_) return;
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    traj_id_ = msg->traj_id;
    exec_time_ = 0.0;
    last_update_time_ = now();
    receive_traj_ = true;
    trajectory_stamp_=msg->start_time;
    stationary_trajectory_=(points.colwise()-points.col(0)).colwise().norm().maxCoeff()<1e-8;
    if(obstacle_stop_) {
      recovery_trajectory_=!stationary_trajectory_ && trajectory_stamp_>=emergency_stamp_;
      if(recovery_trajectory_ && planner_executing_) obstacle_stop_=false;
    }
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
    odom_stamp_ = msg->header.stamp;
    have_odom_ = true;
  }

  void cmdCallback()
  {
    if (task_id_.empty() || !have_odom_ || (now() - odom_stamp_).seconds() > odom_timeout_) {
      publishStop(); publishExecutionFrozen(false);
      publishState(task_id_.empty() ? "IDLE" : "ODOM_STALE"); return;
    }
    if (obstacle_stop_) {
      publishStop();publishExecutionFrozen(false);publishState("OBSTACLE_STOP");
      last_update_time_=now();return;
    }
    if (!receive_traj_ || !have_goal_)
    {
      publishExecutionFrozen(false);
      publishStop();
      publishState("WAIT_TRAJECTORY");
      return;
    }
    const auto current_time = now();
    double dt = (current_time - last_update_time_).seconds();
    if (dt < 0.0 || dt > 0.2) dt = 0.0;
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    if (exec_time_ >= traj_duration_ && (pos_des.head<2>() - odom_pos_.head<2>()).norm() < finish_dist_ &&
        (goal_position_ - odom_pos_.head<2>()).norm() < finish_dist_) {
      const double error = normalizeAngle(goal_yaw_ - odom_yaw_);
      publishExecutionFrozen(false);
      publishStop(std::abs(error) <= finish_yaw_ ? 0.0 : kp_yaw_ * error);
      publishState(std::abs(error) <= finish_yaw_ ? "SUCCEEDED" : "ALIGNING");
      last_update_time_ = current_time; return;
    }
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      publishState("TRACKING");
      last_update_time_ = current_time;
      return;
    }

    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    last_update_time_ = current_time;
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(
        Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error,
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_ &&
        (goal_position_ - odom_pos_.head<2>()).norm() < finish_dist_)
      command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
    publishState("TRACKING");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<quadruped_interfaces::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double exec_time_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double max_vx_, max_vy_, max_vyaw_, finish_dist_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
