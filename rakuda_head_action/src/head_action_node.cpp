#include <cmath>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <type_traits>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/point_head.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <urdf/model.h>

using PointHead = control_msgs::action::PointHead;
using FollowJT  = control_msgs::action::FollowJointTrajectory;

// ---- Trait: detect if ResultT has member pointing_angle_error ----
template <typename T, typename = void>
struct has_pointing_angle_error : std::false_type {};

template <typename T>
struct has_pointing_angle_error<T, std::void_t<decltype(std::declval<T>().pointing_angle_error)>>
  : std::true_type {};

// ---- Helper template: safe set ----
template <typename ResultT>
static inline void maybe_set_pointing_angle_error(ResultT & res, double err)
{
  if constexpr (has_pointing_angle_error<ResultT>::value) {
    res.pointing_angle_error = err;
  } else {
    (void)res;
    (void)err;
  }
}

static builtin_interfaces::msg::Duration to_duration_msg(double seconds)
{
  if (seconds < 0.0) seconds = 0.0;
  const int64_t ns = static_cast<int64_t>(seconds * 1e9);

  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(ns / 1000000000LL);
  d.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return d;
}

static double clamp_pi(double a)
{
  while (a > M_PI)  a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double clamp(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

class RakudaHeadActionNode : public rclcpp::Node
{
public:
  RakudaHeadActionNode()
  : Node("head_action"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ---- Parameters
    action_name_ = this->declare_parameter<std::string>("action_name", "/point_head");
    traj_action_name_ = this->declare_parameter<std::string>(
      "trajectory_action_name", "/head_controller/follow_joint_trajectory");

    reference_frame_ = this->declare_parameter<std::string>("reference_frame", "torso_link");

    yaw_joint_   = this->declare_parameter<std::string>("yaw_joint", "neck_yaw_joint");
    pitch_joint_ = this->declare_parameter<std::string>("pitch_joint", "neck_pitch_joint");

    yaw_sign_   = this->declare_parameter<double>("yaw_sign", 1.0);
    pitch_sign_ = this->declare_parameter<double>("pitch_sign", 1.0);
    pitch_offset_rad_ = this->declare_parameter<double>("pitch_offset_rad", 0.0);

    default_min_duration_sec_ = this->declare_parameter<double>("default_min_duration_sec", 1.0);
    default_max_velocity_     = this->declare_parameter<double>("default_max_velocity", 1.5);

    joint_state_topic_ = this->declare_parameter<std::string>("joint_state_topic", "/joint_states");

    robot_description_topic_ = this->declare_parameter<std::string>("robot_description_topic", "/robot_description");
    clamp_limits_ = this->declare_parameter<bool>("clamp_limits", true);

    // ---- Joint state subscriber
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(js_mtx_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          last_js_pos_[msg->name[i]] = msg->position[i];
        }
      });

    // ---- Subscribe URDF from /robot_description topic (transient_local)
    robot_desc_sub_ = this->create_subscription<std_msgs::msg::String>(
      robot_description_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        urdf::Model model;
        if (!model.initString(msg->data)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF from %s", robot_description_topic_.c_str());
          return;
        }

        auto yaw_j   = model.getJoint(yaw_joint_);
        auto pitch_j = model.getJoint(pitch_joint_);

        if (!yaw_j) {
          RCLCPP_WARN(this->get_logger(), "URDF: joint '%s' not found", yaw_joint_.c_str());
        }
        if (!pitch_j) {
          RCLCPP_WARN(this->get_logger(), "URDF: joint '%s' not found", pitch_joint_.c_str());
        }

        bool yaw_ok = false, pitch_ok = false;
        double yl = 0.0, yu = 0.0, pl = 0.0, pu = 0.0;

        // yaw: if CONTINUOUS -> no hard limits (don't clamp)
        if (yaw_j && yaw_j->type == urdf::Joint::REVOLUTE && yaw_j->limits) {
          yl = yaw_j->limits->lower;
          yu = yaw_j->limits->upper;
          yaw_ok = (yl < yu);
        }

        // pitch: usually REVOLUTE with limits
        if (pitch_j && pitch_j->type == urdf::Joint::REVOLUTE && pitch_j->limits) {
          pl = pitch_j->limits->lower;
          pu = pitch_j->limits->upper;
          pitch_ok = (pl < pu);
        }

        {
          std::lock_guard<std::mutex> lk(limits_mtx_);
          yaw_has_limits_ = yaw_ok;
          pitch_has_limits_ = pitch_ok;
          yaw_lower_ = yl; yaw_upper_ = yu;
          pitch_lower_ = pl; pitch_upper_ = pu;
          limits_ready_ = true;
        }

        if (yaw_ok) {
          RCLCPP_INFO(this->get_logger(), "Yaw limits from URDF:   [%.3f, %.3f] rad", yl, yu);
        } else {
          RCLCPP_WARN(this->get_logger(), "Yaw limits not available (likely CONTINUOUS or missing).");
        }

        if (pitch_ok) {
          RCLCPP_INFO(this->get_logger(), "Pitch limits from URDF: [%.3f, %.3f] rad", pl, pu);
        } else {
          RCLCPP_WARN(this->get_logger(), "Pitch limits not available (missing/continuous?).");
        }

        // One-shot: we only need this once
        robot_desc_sub_.reset();
      });

    // ---- Trajectory action client
    traj_client_ = rclcpp_action::create_client<FollowJT>(this, traj_action_name_);

    // ---- PointHead action server
    action_server_ = rclcpp_action::create_server<PointHead>(
      this,
      action_name_,
      std::bind(&RakudaHeadActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RakudaHeadActionNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&RakudaHeadActionNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "rakuda_head_action up.");
    RCLCPP_INFO(get_logger(), "Serving:   %s", action_name_.c_str());
    RCLCPP_INFO(get_logger(), "Forward to:%s", traj_action_name_.c_str());
    RCLCPP_INFO(get_logger(), "URDF from: %s (clamp_limits=%s)",
                robot_description_topic_.c_str(), clamp_limits_ ? "true" : "false");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const PointHead::Goal> goal)
  {
    std::lock_guard<std::mutex> lk(active_mtx_);
    if (active_) {
      RCLCPP_WARN(get_logger(), "Rejecting PointHead goal: another goal is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->target.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting PointHead goal: target.header.frame_id empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->pointing_frame.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting PointHead goal: pointing_frame empty.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PointHead>> /*goal_handle*/)
  {
    std::lock_guard<std::mutex> lk(active_mtx_);
    cancel_requested_ = true;

    if (active_traj_goal_) {
      (void)traj_client_->async_cancel_goal(active_traj_goal_);
      RCLCPP_WARN(get_logger(), "Cancel requested: forwarded cancel to FollowJointTrajectory.");
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PointHead>> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lk(active_mtx_);
      active_ = true;
      cancel_requested_ = false;
      active_traj_goal_.reset();
    }
    std::thread{std::bind(&RakudaHeadActionNode::execute, this, goal_handle)}.detach();
  }

  std::optional<double> get_joint_pos(const std::string& name)
  {
    std::lock_guard<std::mutex> lk(js_mtx_);
    auto it = last_js_pos_.find(name);
    if (it == last_js_pos_.end()) return std::nullopt;
    return it->second;
  }

  void clear_active()
  {
    std::lock_guard<std::mutex> lk(active_mtx_);
    active_ = false;
    cancel_requested_ = false;
    active_traj_goal_.reset();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PointHead>> goal_handle)
  {
    auto result = std::make_shared<PointHead::Result>();
    const auto goal = goal_handle->get_goal();

    // Wait for controller action server
    if (!traj_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "FollowJointTrajectory server not available: %s", traj_action_name_.c_str());
      goal_handle->abort(result);
      clear_active();
      return;
    }

    // Transform target -> reference_frame_
    geometry_msgs::msg::PointStamped target_in_ref;
    try {
      target_in_ref = tf_buffer_.transform(goal->target, reference_frame_, tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "TF failed (%s -> %s): %s",
                   goal->target.header.frame_id.c_str(), reference_frame_.c_str(), ex.what());
      goal_handle->abort(result);
      clear_active();
      return;
    }

    // Compute yaw/pitch in reference frame (x forward, y left, z up)
    const double x = target_in_ref.point.x;
    const double y = target_in_ref.point.y;
    const double z = target_in_ref.point.z;

    const double yaw   = yaw_sign_   * std::atan2(y, x);
    const double horiz = std::sqrt(x*x + y*y);
    const double pitch = pitch_sign_ * std::atan2(-z, horiz) + pitch_offset_rad_;

    double yaw_cmd   = clamp_pi(yaw);
    double pitch_cmd = clamp_pi(pitch);

    // Clamp to URDF limits (if available)
    if (clamp_limits_) {
      std::lock_guard<std::mutex> lk(limits_mtx_);
      if (limits_ready_) {
        if (yaw_has_limits_) {
          yaw_cmd = clamp(yaw_cmd, yaw_lower_, yaw_upper_);
        }
        if (pitch_has_limits_) {
          pitch_cmd = clamp(pitch_cmd, pitch_lower_, pitch_upper_);
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "Joint limits not loaded yet from %s; sending unclamped goals.",
          robot_description_topic_.c_str());
      }
    }

    // Duration selection
    double min_dur = default_min_duration_sec_;
    {
      const rclcpp::Duration md(goal->min_duration);
      const double md_s = md.seconds();
      if (md_s > 1e-9) min_dur = md_s;
    }

    double max_vel = default_max_velocity_;
    if (goal->max_velocity > 0.0) max_vel = goal->max_velocity;

    double dur = min_dur;
    auto yaw_cur_opt = get_joint_pos(yaw_joint_);
    auto pit_cur_opt = get_joint_pos(pitch_joint_);
    if (max_vel > 0.0 && yaw_cur_opt && pit_cur_opt) {
      const double dy = std::fabs(clamp_pi(yaw_cmd - *yaw_cur_opt));
      const double dp = std::fabs(clamp_pi(pitch_cmd - *pit_cur_opt));
      const double needed = std::max(dy, dp) / max_vel;
      if (needed > dur) dur = needed;
      if (dur < min_dur) dur = min_dur;
    }

    // Build FollowJointTrajectory goal (single point)
    FollowJT::Goal jt_goal;
    jt_goal.trajectory.joint_names = {yaw_joint_, pitch_joint_};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {yaw_cmd, pitch_cmd};
    pt.time_from_start = to_duration_msg(dur);
    jt_goal.trajectory.points.push_back(pt);

    // Wait result WITHOUT spinning a second executor
    std::mutex wait_mtx;
    std::condition_variable wait_cv;
    bool got_result = false;
    rclcpp_action::ResultCode traj_code = rclcpp_action::ResultCode::UNKNOWN;
    bool goal_rejected = false;

    rclcpp_action::Client<FollowJT>::SendGoalOptions opts;

    opts.goal_response_callback =
      [this, &wait_mtx, &wait_cv, &got_result, &traj_code, &goal_rejected]
      (std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowJT>> gh)
      {
        {
          std::lock_guard<std::mutex> lk(active_mtx_);
          active_traj_goal_ = gh;
        }
        if (!gh) {
          // rejected: wake up execute thread
          {
            std::lock_guard<std::mutex> lk(wait_mtx);
            goal_rejected = true;
            got_result = true;
            traj_code = rclcpp_action::ResultCode::ABORTED;
          }
          wait_cv.notify_one();
        }
      };

    opts.result_callback =
      [&wait_mtx, &wait_cv, &got_result, &traj_code]
      (const rclcpp_action::ClientGoalHandle<FollowJT>::WrappedResult & wr)
      {
        {
          std::lock_guard<std::mutex> lk(wait_mtx);
          got_result = true;
          traj_code = wr.code;
        }
        wait_cv.notify_one();
      };

    (void)traj_client_->async_send_goal(jt_goal, opts);

    // Wait loop (also reacts to cancel)
    {
      std::unique_lock<std::mutex> lk(wait_mtx);
      while (!got_result && rclcpp::ok()) {
        {
          std::lock_guard<std::mutex> lk2(active_mtx_);
          if (cancel_requested_) {
            if (active_traj_goal_) {
              (void)traj_client_->async_cancel_goal(active_traj_goal_);
            }
            goal_handle->canceled(result);
            clear_active();
            return;
          }
        }
        wait_cv.wait_for(lk, std::chrono::milliseconds(100));
      }
    }

    if (goal_rejected) {
      RCLCPP_ERROR(get_logger(), "FollowJointTrajectory goal rejected by server.");
      goal_handle->abort(result);
      clear_active();
      return;
    }

    const bool ok = (traj_code == rclcpp_action::ResultCode::SUCCEEDED);

    // Compute approx error and fill result (optionally)
    double err = 0.0;
    auto yaw_end = get_joint_pos(yaw_joint_);
    auto pit_end = get_joint_pos(pitch_joint_);
    if (yaw_end && pit_end) {
      const double dy = clamp_pi(yaw_cmd - *yaw_end);
      const double dp = clamp_pi(pitch_cmd - *pit_end);
      err = std::sqrt(dy*dy + dp*dp);
    }
    maybe_set_pointing_angle_error(*result, err);

    if (ok) {
      goal_handle->succeed(result);
    } else {
      RCLCPP_ERROR(get_logger(), "FollowJointTrajectory failed (code=%d).", (int)traj_code);
      goal_handle->abort(result);
    }

    clear_active();
  }

private:
  // Params
  std::string action_name_;
  std::string traj_action_name_;
  std::string reference_frame_;
  std::string yaw_joint_;
  std::string pitch_joint_;
  std::string joint_state_topic_;

  std::string robot_description_topic_;
  bool clamp_limits_{true};

  double yaw_sign_{1.0};
  double pitch_sign_{1.0};
  double pitch_offset_rad_{0.0};
  double default_min_duration_sec_{1.0};
  double default_max_velocity_{1.5};

  // Action server/client
  rclcpp_action::Server<PointHead>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJT>::SharedPtr traj_client_;

  // Joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::mutex js_mtx_;
  std::unordered_map<std::string, double> last_js_pos_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // URDF robot_description subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  // Limits from URDF
  std::mutex limits_mtx_;
  bool limits_ready_{false};
  bool yaw_has_limits_{false};
  bool pitch_has_limits_{false};
  double yaw_lower_{0.0}, yaw_upper_{0.0};
  double pitch_lower_{0.0}, pitch_upper_{0.0};

  // Active/cancel
  std::mutex active_mtx_;
  bool active_{false};
  bool cancel_requested_{false};
  std::shared_ptr<rclcpp_action::ClientGoalHandle<FollowJT>> active_traj_goal_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RakudaHeadActionNode>());
  rclcpp::shutdown();
  return 0;
}


