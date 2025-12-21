#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/point_head.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class RakudaHeadActionNode : public rclcpp::Node
{
public:
  using PointHead = control_msgs::action::PointHead;
  using GoalHandlePH = rclcpp_action::ServerGoalHandle<PointHead>;

  RakudaHeadActionNode()
  : Node("rakuda_head_action_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    controller_name_ = declare_parameter<std::string>("controller_name", "head_controller");
    action_name_     = declare_parameter<std::string>("action_name", "point_head_action");

    root_frame_      = declare_parameter<std::string>("root_frame", "torso_link");

    yaw_joint_       = declare_parameter<std::string>("yaw_joint", "neck_yaw_joint");
    pitch_joint_     = declare_parameter<std::string>("pitch_joint", "neck_pitch_joint");

    // IMPORTANT for point (3): we also need the pitch link frame name
    pitch_link_      = declare_parameter<std::string>("pitch_link", "neck_pitch_link");

    // Default pointing frame = camera optical frame
    default_pointing_frame_ = declare_parameter<std::string>(
      "default_pointing_frame", "camera_color_optical_frame");

    // Joint limits (keep yours here)
    yaw_min_   = declare_parameter<double>("yaw_min",   -3.14159);
    yaw_max_   = declare_parameter<double>("yaw_max",    3.14159);
    pitch_min_ = declare_parameter<double>("pitch_min", -1.5708);
    pitch_max_ = declare_parameter<double>("pitch_max",  1.5708);

    default_duration_sec_ = declare_parameter<double>("default_duration_sec", 0.8);
    tf_timeout_sec_       = declare_parameter<double>("tf_timeout_sec", 0.5);

    // Small iterative solver gains/iterations
    solve_iterations_ = declare_parameter<int>("solve_iterations", 10);
    solve_gain_yaw_   = declare_parameter<double>("solve_gain_yaw", 0.9);
    solve_gain_pitch_ = declare_parameter<double>("solve_gain_pitch", 0.9);

    cmd_topic_ = "/" + controller_name_ + "/joint_trajectory";
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(cmd_topic_, 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      std::bind(&RakudaHeadActionNode::on_joint_state, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<PointHead>(
      this,
      action_name_,
      std::bind(&RakudaHeadActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RakudaHeadActionNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&RakudaHeadActionNode::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "rakuda_head_action_node up.");
    RCLCPP_INFO(get_logger(), "Action: /%s", action_name_.c_str());
    RCLCPP_INFO(get_logger(), "Command topic: %s", cmd_topic_.c_str());
    RCLCPP_INFO(get_logger(), "root_frame=%s pitch_link=%s default_pointing_frame=%s",
                root_frame_.c_str(), pitch_link_.c_str(), default_pointing_frame_.c_str());
  }

private:
  // ---- Joint states cache
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(js_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) {
        joint_pos_[msg->name[i]] = msg->position[i];
      }
    }
  }

  bool get_current_positions(double & yaw, double & pitch)
  {
    std::lock_guard<std::mutex> lk(js_mutex_);
    auto iy = joint_pos_.find(yaw_joint_);
    auto ip = joint_pos_.find(pitch_joint_);
    if (iy == joint_pos_.end() || ip == joint_pos_.end()) return false;
    yaw = iy->second;
    pitch = ip->second;
    return true;
  }

  // ---- Action callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PointHead::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePH>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePH> gh)
  {
    std::thread([this, gh]() { execute(gh); }).detach();
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(hi, v));
  }

  void publish_trajectory(double yaw_cmd, double pitch_cmd, double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    traj.joint_names = {yaw_joint_, pitch_joint_};

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {yaw_cmd, pitch_cmd};
    pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    traj.points = {pt};
    traj_pub_->publish(traj);
  }

  static double angle_between(const tf2::Vector3 & a, const tf2::Vector3 & b)
  {
    const double la = a.length();
    const double lb = b.length();
    if (la < 1e-9 || lb < 1e-9) return 0.0;
    double c = (a.dot(b)) / (la * lb);
    c = std::max(-1.0, std::min(1.0, c));
    return std::acos(c);
  }

  // Convert pointing_axis (expressed in pointing_frame) into pitch_link frame.
  bool get_axis_in_pitch_link(const std::string & pointing_frame,
                              const tf2::Vector3 & axis_in_pointing_frame,
                              tf2::Vector3 & axis_in_pitch_link)
  {
    // lookupTransform(target, source): transforms data FROM source TO target
    // We want axis expressed in pointing_frame -> expressed in pitch_link
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        pitch_link_, pointing_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(),
        "TF lookup failed (target=%s, source=%s): %s",
        pitch_link_.c_str(), pointing_frame.c_str(), ex.what());
      return false;
    }

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    axis_in_pitch_link = tf2::quatRotate(q, axis_in_pointing_frame);
    return axis_in_pitch_link.length() > 1e-9;
  }

  // Compute camera origin and pointing axis in root_frame (current pose).
  bool get_current_camera_axis_in_root(const std::string & pointing_frame,
                                       const tf2::Vector3 & axis_in_pointing_frame,
                                       tf2::Vector3 & cam_origin_root,
                                       tf2::Vector3 & cam_axis_root)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        root_frame_, pointing_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(),
        "TF lookup failed (target=%s, source=%s): %s",
        root_frame_.c_str(), pointing_frame.c_str(), ex.what());
      return false;
    }

    cam_origin_root = tf2::Vector3(
      tf.transform.translation.x,
      tf.transform.translation.y,
      tf.transform.translation.z);

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    cam_axis_root = tf2::quatRotate(q, axis_in_pointing_frame);
    return cam_axis_root.length() > 1e-9;
  }

  // Iteratively solve yaw/pitch so that:
  //   camera_axis_root(yaw,pitch) aligns to direction d_root
  // using a simplified model:
  //   root -> yaw about Z, then pitch about Y (in yaw frame), then fixed axis a0 in pitch_link
  bool solve_yaw_pitch_for_axis(const tf2::Vector3 & a0_in_pitch_link,
                                const tf2::Vector3 & d_root,
                                double yaw_seed, double pitch_seed,
                                double & yaw_out, double & pitch_out)
  {
    tf2::Vector3 a0 = a0_in_pitch_link.normalized();
    tf2::Vector3 d  = d_root.normalized();

    double yaw = yaw_seed;
    double pitch = pitch_seed;

    for (int i = 0; i < solve_iterations_; ++i) {
      // Rz(yaw)
      tf2::Matrix3x3 Rz;
      Rz.setRPY(0.0, 0.0, yaw);

      // Ry(pitch)
      tf2::Matrix3x3 Ry;
      Ry.setRPY(0.0, pitch, 0.0);

      // v = Rz * Ry * a0
      tf2::Vector3 v = Rz * (Ry * a0);
      if (v.length() < 1e-9) break;
      v.normalize();

      // error axis
      tf2::Vector3 e = v.cross(d);  // axis of rotation from v to d (small-angle)
      if (e.length() < 1e-9) break;

      // yaw axis is Z in root
      tf2::Vector3 yaw_axis_root(0, 0, 1);

      // pitch axis is Y in yaw frame -> in root it's Rz * (0,1,0)
      tf2::Vector3 pitch_axis_root = Rz * tf2::Vector3(0, 1, 0);

      double dyaw   = solve_gain_yaw_   * e.dot(yaw_axis_root);
      double dpitch = solve_gain_pitch_ * e.dot(pitch_axis_root);

      yaw   += dyaw;
      pitch += dpitch;

      yaw   = clamp(yaw,   yaw_min_,   yaw_max_);
      pitch = clamp(pitch, pitch_min_, pitch_max_);

      // stop if angle already small
      if (angle_between(v, d) < 1e-3) break;
    }

    yaw_out = yaw;
    pitch_out = pitch;
    return true;
  }

  void execute(const std::shared_ptr<GoalHandlePH> gh)
  {
    auto result = std::make_shared<PointHead::Result>();
    const auto goal = gh->get_goal();

    // Decide pointing_frame & axis (POINT 3)
    std::string pointing_frame = goal->pointing_frame;
    if (pointing_frame.empty()) pointing_frame = default_pointing_frame_;

    tf2::Vector3 axis(goal->pointing_axis.x, goal->pointing_axis.y, goal->pointing_axis.z);
    if (axis.length() < 0.1) {
      // Optical frame default: +Z
      axis = tf2::Vector3(0, 0, 1);
    } else {
      axis.normalize();
    }

    // Transform target to root_frame
    geometry_msgs::msg::PointStamped target_root;
    try {
      target_root = tf_buffer_.transform(goal->target, root_frame_, tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF transform target -> %s failed: %s", root_frame_.c_str(), ex.what());
      gh->abort(result);
      return;
    }

    const tf2::Vector3 target_in_root(target_root.point.x, target_root.point.y, target_root.point.z);

    // Get current camera origin+axis in root (for feedback + direction computation)
    tf2::Vector3 cam_origin_root, cam_axis_root;
    if (!get_current_camera_axis_in_root(pointing_frame, axis, cam_origin_root, cam_axis_root)) {
      gh->abort(result);
      return;
    }

    tf2::Vector3 d_root = target_in_root - cam_origin_root;
    if (d_root.length() < 1e-6) {
      RCLCPP_ERROR(get_logger(), "Target too close to camera origin; cannot define direction.");
      gh->abort(result);
      return;
    }

    // Publish initial feedback: current pointing error wrt camera axis
    {
      auto fb = std::make_shared<PointHead::Feedback>();
      fb->pointing_angle_error = angle_between(cam_axis_root, d_root);
      gh->publish_feedback(fb);
    }

    // Convert pointing axis to pitch_link frame (fixed vector a0)
    tf2::Vector3 a0_pitch;
    if (!get_axis_in_pitch_link(pointing_frame, axis, a0_pitch)) {
      gh->abort(result);
      return;
    }
    a0_pitch.normalize();

    // Seed from current joints (if available)
    double yaw_now = 0.0, pitch_now = 0.0;
    (void)get_current_positions(yaw_now, pitch_now);

    // Solve yaw/pitch to align the *camera optical axis* to target direction
    double yaw_cmd = yaw_now;
    double pitch_cmd = pitch_now;
    solve_yaw_pitch_for_axis(a0_pitch, d_root, yaw_now, pitch_now, yaw_cmd, pitch_cmd);

    // Duration handling (min_duration + max_velocity)
    double duration = std::max(default_duration_sec_,
                               rclcpp::Duration(goal->min_duration).seconds());

    if (goal->max_velocity > 1e-6) {
      const double dy = std::fabs(yaw_cmd - yaw_now);
      const double dp = std::fabs(pitch_cmd - pitch_now);
      duration = std::max(duration, std::max(dy, dp) / goal->max_velocity);
    }

    // Send command
    publish_trajectory(yaw_cmd, pitch_cmd, duration);

    // For now we succeed immediately (we'll do point 1-2 next)
    gh->succeed(result);
  }

private:
  std::string controller_name_;
  std::string action_name_;
  std::string root_frame_;
  std::string yaw_joint_;
  std::string pitch_joint_;
  std::string pitch_link_;
  std::string default_pointing_frame_;

  double yaw_min_{-3.14159}, yaw_max_{3.14159};
  double pitch_min_{-1.5708}, pitch_max_{1.5708};

  double default_duration_sec_{0.8};
  double tf_timeout_sec_{0.5};

  int solve_iterations_{10};
  double solve_gain_yaw_{0.9};
  double solve_gain_pitch_{0.9};

  std::string cmd_topic_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;

  std::mutex js_mutex_;
  std::unordered_map<std::string, double> joint_pos_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::Server<PointHead>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RakudaHeadActionNode>());
  rclcpp::shutdown();
  return 0;
}

