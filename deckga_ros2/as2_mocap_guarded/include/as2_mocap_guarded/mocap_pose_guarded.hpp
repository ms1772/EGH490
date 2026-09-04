// Copyright 2024 Universidad Politecnica de Madrid
// Copyright 2026 Mitchell Solomon (hardened fork)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politecnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
* @file mocap_pose_guarded.hpp
*
* S1 -- a hardened fork of the Aerostack2 `mocap_pose` state-estimator plugin
* (as2_state_estimator 1.1.3, /opt/ros/humble/include/mocap_pose.hpp).
*
* This plugin is a DROP-IN REPLACEMENT for `mocap_pose`, not a patch of it: the
* stock package is left untouched. Everything the original did -- ENU pass-through
* with no frame conversion, identity map->odom, odom->base broadcast, pose and
* twist publication, the orientation LERP -- is preserved. Four defects are fixed
* and one health topic is added.
*
* D1  ORIGIN POSE ON NAME MISMATCH
*     Original: rigid_bodies_callback() scans msg->rigidbodies for
*     rigid_body_name and then calls process_mocap_pose(pose_msg)
*     UNCONDITIONALLY, outside the loop. geometry_msgs/Quaternion defaults to
*     w = 1, so a miss publishes a perfectly well-formed pose at (0, 0, 0) with
*     identity orientation, at full mocap rate, silently. In a netted 8x6x4 m
*     volume a drone that believes it is at the origin flies into the net.
*     Here: a miss DROPS the message (early return, no pose, no TF, no twist)
*     and logs a throttled RCLCPP_ERROR naming the body it wanted and listing
*     every name that was actually in the array.
*
* D2  FRAME ORIGIN SET BY A STARTUP RACE
*     Original: earth_to_map_ is latched from the FIRST message ever received and
*     never revised (has_earth_to_map_). Whether the estimator starts before or
*     after Motive acquires the body decides whether earth->map is identity or
*     that airframe's resting pose INCLUDING ITS YAW. Three drones started
*     independently can end up with three different world origins in one session,
*     with no log line saying so. Upstream flags it itself:
*       "TODO(javilinos): MODIFY this to a initial earth to map transform
*        (reading initial position from parameters or msgs)"
*     Here: earth->map comes from parameters, DEFAULTING TO IDENTITY, and is
*     published once at setup before any mocap data is seen. earth == map == the
*     lab frame, deterministically, for every drone, regardless of start order.
*
* D3  VELOCITY NOISE / STATIC LOCALS
*     Original: twist is a raw finite difference and twist_smooth_filter_cte
*     defaults to 1.0, i.e. the filter is OFF. ~2 mm mocap noise at 100 Hz is
*     ~0.28 m/s of velocity noise fed straight to a speed controller. The
*     original also keeps `static` locals inside a member function, which are
*     shared process-wide across every instance of the class.
*     Here: the default is 0.2 (see the derivation in on_setup) and the statics
*     are per-instance members.
*
* D4  NO INPUT VALIDATION
*     Original: whatever Motive/VRPN emitted was used. Here: non-finite
*     positions/orientations and non-unit quaternions are counted, logged
*     throttled, and DROPPED. A rejected sample is never replaced by the last
*     good pose -- bad data must age out exactly like no data, so that the
*     downstream freshness check is the single source of truth about liveness.
*
* D5  SILENT QoS INCOMPATIBILITY  (found by running it, not by reading it)
*     Original: subscribes with rclcpp::QoS(10), i.e. RELIABLE. Both mocap4r2's
*     own drivers and this project's flight_ops/nodes/vrpn_to_rigidbodies.py
*     publish /mocap/rigid_bodies with BEST_EFFORT sensor QoS, which is the ROS
*     convention for a 100 Hz sensor stream. A reliable subscriber and a
*     best-effort publisher are INCOMPATIBLE: DDS matches nothing, not one
*     message is delivered, and the estimator sits there looking healthy with an
*     empty pose topic. Here: the reliability is a parameter, defaulting to
*     best_effort so it matches the sources this stack actually has, and an
*     incompatible-QoS event callback turns the silence into a named error line.
*     Set mocap_qos_reliability:=reliable to reproduce upstream exactly.
*
* H   HEALTH TOPIC
*     /{ns}/mocap_health, std_msgs/String carrying JSON, published on a TIMER so
*     that it keeps reporting while the mocap feed is silent. See
*     publish_health() for the full field list and the choice of message type.
*
* @authors David Perez Saura, Rafael Perez Segui, Javier Melero Deza,
*          Miguel Fernandez Cortizas, Pedro Arias Perez  (original mocap_pose)
*          Mitchell Solomon  (S1 hardening)
*/

#ifndef AS2_MOCAP_GUARDED__MOCAP_POSE_GUARDED_HPP_
#define AS2_MOCAP_GUARDED__MOCAP_POSE_GUARDED_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mocap4r2_msgs/msg/rigid_bodies.hpp>
#include <std_msgs/msg/string.hpp>

#include "as2_state_estimator/plugin_base.hpp"

namespace mocap_pose_guarded
{

/**
 * @brief Wall-clock rate limiter for log lines.
 *
 * Deliberately NOT RCLCPP_*_THROTTLE. Those macros throttle on a ROS clock; with
 * use_sim_time:=true and no /clock publisher the ROS clock sits at t = 0 forever,
 * so the macro emits exactly one line and then goes quiet for the rest of the
 * session -- the opposite of what a fault log is for. std::steady_clock always
 * advances and is immune to sim time, bag playback and NTP steps. It also lets
 * the caller skip building the message string when it would be dropped.
 */
class LogThrottle
{
public:
  explicit LogThrottle(double period_s)
  : period_s_(period_s) {}

  bool ready()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!primed_ || std::chrono::duration<double>(now - last_).count() >= period_s_) {
      primed_ = true;
      last_ = now;
      return true;
    }
    return false;
  }

private:
  double period_s_;
  bool primed_ = false;
  std::chrono::steady_clock::time_point last_;
};

/// Outcome of validating one mocap sample.
enum class SampleStatus
{
  OK,
  NON_FINITE,
  NON_UNIT_QUATERNION
};

class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase
{
  // ---- subscriptions / publishers -----------------------------------------
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr rigid_bodies_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  // ---- frames --------------------------------------------------------------
  // D2: fixed at setup from parameters; never latched from incoming data.
  tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
  const tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();  // ALWAYS IDENTITY
  tf2::Transform odom_to_base_ = tf2::Transform::getIdentity();

  // ---- parameters ----------------------------------------------------------
  std::string mocap_topic_;
  std::string rigid_body_name_;
  std::string mocap_qos_reliability_ = "best_effort";
  double twist_alpha_ = 0.2;
  double orientation_alpha_ = 1.0;
  double earth_to_map_x_ = 0.0;
  double earth_to_map_y_ = 0.0;
  double earth_to_map_z_ = 0.0;
  double earth_to_map_yaw_ = 0.0;
  double quaternion_tolerance_ = 1e-3;
  double mocap_timeout_ = 0.25;
  std::string health_topic_ = "mocap_health";
  double health_rate_ = 2.0;
  double log_throttle_period_ = 2.0;

  // ---- state (D3: these were `static` locals in the original) --------------
  geometry_msgs::msg::PoseStamped last_pose_msg_;
  tf2::Vector3 last_position_ = tf2::Vector3(0.0, 0.0, 0.0);
  bool has_last_position_ = false;
  geometry_msgs::msg::TwistStamped twist_body_msg_;

  // ---- health counters -----------------------------------------------------
  uint64_t messages_ = 0;              ///< RigidBodies messages received
  uint64_t accepted_ = 0;              ///< samples that produced a pose
  uint64_t name_misses_ = 0;           ///< D1: configured body absent from array
  uint64_t rejects_non_finite_ = 0;    ///< NaN/Inf in position or orientation
  uint64_t rejects_quaternion_ = 0;    ///< |q| further than tolerance from 1
  rclcpp::Time last_good_time_;
  bool has_last_good_ = false;
  std::string last_names_seen_ = "";   ///< names in the most recent name-miss

  LogThrottle miss_throttle_{2.0};
  LogThrottle reject_throttle_{2.0};
  LogThrottle dt_throttle_{2.0};

public:
  Plugin()
  : as2_state_estimator_plugin_base::StateEstimatorBase() {}

  void on_setup() override
  {
    // -- parameters ---------------------------------------------------------
    // The stock as2_state_estimator node runs with
    // automatically_declare_parameters_from_overrides(true), so a parameter only
    // exists if it appeared in a yaml/CLI override. get_*_param below declares
    // the parameter itself when it is absent, which is what gives every new knob
    // a real code-level default instead of a ParameterNotDeclaredException.
    mocap_topic_ = get_string_param("mocap_topic", "/mocap/rigid_bodies");
    if (mocap_topic_.empty()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Parameter 'mocap_topic' not set");
      throw std::runtime_error("Parameter 'mocap_topic' not set");
    }

    // Intentionally NO usable default. A wrong or absent rigid_body_name is the
    // exact failure D1 is about; refusing to start is the safe outcome.
    rigid_body_name_ = get_string_param("rigid_body_name", "");
    if (rigid_body_name_.empty()) {
      RCLCPP_ERROR(
        node_ptr_->get_logger(),
        "Parameter 'rigid_body_name' not set. It must match the rigid body name in Motive "
        "EXACTLY (case and punctuation included). Refusing to start.");
      throw std::runtime_error("Parameter 'rigid_body_name' not set");
    }

    // D3: 0.2, not the upstream 1.0 (== filter disabled).
    //
    // Derivation for a 100 Hz OptiTrack feed with ~2 mm 1-sigma static position
    // noise (the number the I-02 bench card expects from the O-134 rig):
    //   raw finite difference   sigma_v = sigma_p * sqrt(2) * f_s
    //                                   = 0.002 * 1.414 * 100 ~= 0.28 m/s
    //   one-pole IIR gain on white noise = sqrt(alpha / (2 - alpha))
    //   alpha = 0.2  ->  sqrt(0.2/1.8) = 0.333  ->  sigma_v ~= 0.094 m/s  (3x better)
    //   -3 dB corner  f_c ~= f_s * alpha / (2*pi) = 100 * 0.2 / 6.283 ~= 3.2 Hz
    //   group delay   ~= (1 - alpha) / (alpha * f_s) = 0.8 / 20 = 40 ms
    // A multirotor position/velocity loop runs at roughly 0.5-1.5 Hz bandwidth,
    // so a 3.2 Hz corner passes the whole control-relevant band while cutting the
    // differentiation noise threefold, and 40 ms of lag is small against the loop
    // period. alpha = 0.1 would halve the noise again but costs 90 ms of lag at a
    // 1.6 Hz corner -- inside the position loop, so it would show up as sluggish
    // velocity tracking. 0.2 is the compromise. Set 1.0 to reproduce upstream.
    twist_alpha_ = get_double_param("twist_smooth_filter_cte", 0.2);

    // Left at the upstream default (1.0 == off) ON PURPOSE. The original blends
    // quaternion components linearly and does NOT renormalise the result, so any
    // value below 1.0 emits a non-unit quaternion. Fixing that properly means
    // slerp, which is a behaviour change beyond this fork's scope; keeping the
    // default at 1.0 makes the blend an exact no-op.
    orientation_alpha_ = get_double_param("orientation_smooth_filter_cte", 1.0);

    // D2: earth->map from parameters, identity by default.
    earth_to_map_x_ = get_double_param("earth_to_map_x", 0.0);
    earth_to_map_y_ = get_double_param("earth_to_map_y", 0.0);
    earth_to_map_z_ = get_double_param("earth_to_map_z", 0.0);
    earth_to_map_yaw_ = get_double_param("earth_to_map_yaw", 0.0);

    quaternion_tolerance_ = get_double_param("quaternion_tolerance", 1e-3);
    mocap_timeout_ = get_double_param("mocap_timeout", 0.25);
    health_topic_ = get_string_param("mocap_health_topic", "mocap_health");
    health_rate_ = get_double_param("mocap_health_rate", 2.0);
    log_throttle_period_ = get_double_param("log_throttle_period", 2.0);

    miss_throttle_ = LogThrottle(log_throttle_period_);
    reject_throttle_ = LogThrottle(log_throttle_period_);
    dt_throttle_ = LogThrottle(log_throttle_period_);

    last_good_time_ = node_ptr_->now();
    has_last_good_ = false;

    // -- subscriptions ------------------------------------------------------
    // D5: upstream hard-codes rclcpp::QoS(10), which is RELIABLE, and a reliable
    // subscriber never matches a best-effort publisher. mocap4r2's drivers and
    // this project's vrpn_to_rigidbodies.py both publish best-effort, so the
    // default here is best_effort; "reliable" restores upstream behaviour.
    mocap_qos_reliability_ = get_string_param("mocap_qos_reliability", "best_effort");
    rclcpp::QoS mocap_qos(10);
    if (mocap_qos_reliability_ == "reliable") {
      mocap_qos.reliable();
    } else if (mocap_qos_reliability_ == "system_default") {
      // leave the RMW default in place
    } else {
      if (mocap_qos_reliability_ != "best_effort") {
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Unknown mocap_qos_reliability '%s' (expected best_effort|reliable|system_default); "
          "using best_effort", mocap_qos_reliability_.c_str());
        mocap_qos_reliability_ = "best_effort";
      }
      mocap_qos.best_effort();
    }

    rclcpp::SubscriptionOptions sub_options;
    // Without this, a QoS mismatch is completely silent on the subscriber side:
    // the topic simply never delivers, which looks identical to "mocap is down".
    sub_options.event_callbacks.incompatible_qos_callback =
      [this](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "MOCAP QoS INCOMPATIBLE on '%s': this subscription's QoS cannot match the publisher "
          "(offending policy: %s; %d incompatible publisher(s) total). NO messages will ever be "
          "delivered on this topic. Set the 'mocap_qos_reliability' parameter "
          "(best_effort|reliable|system_default) to match the source.",
          mocap_topic_.c_str(), qos_policy_name(info.last_policy_kind), info.total_count);
      };

    rigid_bodies_sub_ = node_ptr_->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      mocap_topic_, mocap_qos,
      std::bind(&Plugin::rigid_bodies_callback, this, std::placeholders::_1),
      sub_options);

    // -- health -------------------------------------------------------------
    // QoS: depth 10, reliable, plus transient_local so a monitor that attaches
    // late (preflight_check.py, volume_guard.py) immediately gets the current
    // state instead of waiting up to 1/health_rate_ for the next tick.
    const auto health_qos = rclcpp::QoS(10).transient_local().reliable();
    health_pub_ = node_ptr_->create_publisher<std_msgs::msg::String>(health_topic_, health_qos);

    const double rate = (health_rate_ > 0.0) ? health_rate_ : 2.0;
    health_timer_ = node_ptr_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)),
      std::bind(&Plugin::publish_health, this));

    // -- static transforms --------------------------------------------------
    // map->odom identity, exactly as upstream.
    const geometry_msgs::msg::TransformStamped map_to_odom =
      as2::tf::getTransformation(get_map_frame(), get_odom_frame(), 0, 0, 0, 0, 0, 0);
    publish_static_transform(map_to_odom);

    // D2: earth->map published NOW, from parameters, before a single mocap
    // sample has been seen. This is the whole fix -- the transform no longer
    // depends on when this process started relative to Motive.
    tf2::Quaternion q_earth_to_map;
    q_earth_to_map.setRPY(0.0, 0.0, earth_to_map_yaw_);
    earth_to_map_ = tf2::Transform(
      q_earth_to_map,
      tf2::Vector3(earth_to_map_x_, earth_to_map_y_, earth_to_map_z_));

    geometry_msgs::msg::TransformStamped earth_to_map_msg;
    earth_to_map_msg.transform = tf2::toMsg(earth_to_map_);
    earth_to_map_msg.header.stamp = node_ptr_->now();
    earth_to_map_msg.header.frame_id = get_earth_frame();
    earth_to_map_msg.child_frame_id = get_map_frame();
    publish_static_transform(earth_to_map_msg);

    const bool identity = (earth_to_map_x_ == 0.0 && earth_to_map_y_ == 0.0 &&
      earth_to_map_z_ == 0.0 && earth_to_map_yaw_ == 0.0);
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "mocap_pose_guarded: tracking rigid body '%s' on '%s'; "
      "earth->map = [x %.3f y %.3f z %.3f yaw %.3f]%s from parameters, NOT from the first "
      "sample; subscription reliability %s; twist_smooth_filter_cte %.3f; "
      "quaternion_tolerance %.1e; mocap_timeout %.3f s; health on '%s' at %.1f Hz",
      rigid_body_name_.c_str(), mocap_topic_.c_str(),
      earth_to_map_x_, earth_to_map_y_, earth_to_map_z_, earth_to_map_yaw_,
      identity ? " (IDENTITY)" : "",
      mocap_qos_reliability_.c_str(),
      twist_alpha_, quaternion_tolerance_, mocap_timeout_,
      health_topic_.c_str(), rate);

    // Emit one health sample immediately so a monitor never has to guess whether
    // the plugin came up at all.
    publish_health();
  }

  /**
   * @brief Report the configured earth->map.
   *
   * The base class default warns and hands back identity regardless of what the
   * plugin actually broadcast. Harmless upstream only because earth->map was a
   * mystery value anyway; here it is a known parameter, so report the truth.
   */
  bool get_earth_to_map_transform(geometry_msgs::msg::TransformStamped & transform) override
  {
    transform.transform = tf2::toMsg(earth_to_map_);
    transform.header.stamp = node_ptr_->now();
    transform.header.frame_id = get_earth_frame();
    transform.child_frame_id = get_map_frame();
    return true;
  }

  /**
   * @brief Velocity from successive positions, with a one-pole smoother.
   *
   * Same maths as upstream. The difference is that `last_position_`,
   * `has_last_position_` and `twist_body_msg_` are MEMBERS. Upstream declares the
   * first and third as `static` locals inside this member function, so they are
   * shared by every Plugin instance in the process: two estimators in one
   * component container would differentiate each other's positions, and the
   * function-local static is initialised exactly once, on the first call, ever.
   */
  const geometry_msgs::msg::TwistStamped & twist_from_pose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::vector<tf2::Transform> * data = nullptr)
  {
    const auto last_time = twist_msg_.header.stamp;
    const auto dt = (rclcpp::Time(pose.header.stamp) - rclcpp::Time(last_time)).seconds();
    if (dt <= 0) {
      if (dt_throttle_.ready()) {
        RCLCPP_WARN(node_ptr_->get_logger(), "dt <= 0 (%.6f s), reusing previous twist", dt);
      }
      return twist_msg_;
    }

    const tf2::Vector3 current_position(
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // First accepted sample of this instance: no previous position exists, so the
    // only honest velocity is zero. Upstream got zero here by accident, via the
    // one-time initialisation of the static local.
    tf2::Vector3 vel(0.0, 0.0, 0.0);
    if (has_last_position_) {
      vel = (current_position - last_position_) / dt;
    }
    last_position_ = current_position;
    has_last_position_ = true;

    vel = twist_alpha_ * vel + (1 - twist_alpha_) * tf2::Vector3(
      twist_msg_.twist.linear.x,
      twist_msg_.twist.linear.y,
      twist_msg_.twist.linear.z);

    twist_msg_.header.stamp = pose.header.stamp;
    twist_msg_.twist.linear.x = vel.x();
    twist_msg_.twist.linear.y = vel.y();
    twist_msg_.twist.linear.z = vel.z();
    // TODO(javilinos): add angular velocity -> this_could_be_obtained_from_imu
    twist_msg_.twist.angular.x = 0;
    twist_msg_.twist.angular.y = 0;
    twist_msg_.twist.angular.z = 0;

    if (data != nullptr && data->size() >= 3) {
      const tf2::Transform & earth_to_map = data->at(0);
      const tf2::Transform & map_to_odom = data->at(1);
      const tf2::Transform & odom_to_base = data->at(2);

      vel = tf2::quatRotate(
        (odom_to_base.inverse() * map_to_odom.inverse() * earth_to_map.inverse()).getRotation(),
        vel);

      twist_body_msg_.header.stamp = pose.header.stamp;
      twist_body_msg_.header.frame_id = get_base_frame();
      twist_body_msg_.twist.linear.x = vel.x();
      twist_body_msg_.twist.linear.y = vel.y();
      twist_body_msg_.twist.linear.z = vel.z();
      twist_body_msg_.twist.angular.x = 0;
      twist_body_msg_.twist.angular.y = 0;
      twist_body_msg_.twist.angular.z = 0;
      return twist_body_msg_;
    }
    return twist_msg_;
  }

  geometry_msgs::msg::TwistStamped twist_msg_;

  // ------------------------------------------------------------------------
  // validation (D4) -- public and static so it can be unit-tested without a node
  // ------------------------------------------------------------------------
  static SampleStatus validate_pose(
    const geometry_msgs::msg::Pose & pose, double tolerance, double & quat_norm)
  {
    quat_norm = std::numeric_limits<double>::quiet_NaN();

    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w))
    {
      return SampleStatus::NON_FINITE;
    }

    quat_norm = std::sqrt(
      pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);

    // Catches the all-zero quaternion (norm 0) and the scaled/garbage quaternion
    // a mocap bridge emits when a marker set is only partially solved.
    if (std::fabs(quat_norm - 1.0) > tolerance) {
      return SampleStatus::NON_UNIT_QUATERNION;
    }
    return SampleStatus::OK;
  }

private:
  /// Human-readable name for the QoS policy that caused an incompatibility.
  static const char * qos_policy_name(rmw_qos_policy_kind_t kind)
  {
    switch (kind) {
      case RMW_QOS_POLICY_DURABILITY: return "DURABILITY";
      case RMW_QOS_POLICY_DEADLINE: return "DEADLINE";
      case RMW_QOS_POLICY_LIVELINESS: return "LIVELINESS";
      case RMW_QOS_POLICY_RELIABILITY: return "RELIABILITY";
      case RMW_QOS_POLICY_HISTORY: return "HISTORY";
      case RMW_QOS_POLICY_LIFESPAN: return "LIFESPAN";
      default: return "UNKNOWN";
    }
  }

  // ------------------------------------------------------------------------
  // parameter helpers
  // ------------------------------------------------------------------------
  std::string get_string_param(const std::string & name, const std::string & fallback)
  {
    if (!node_ptr_->has_parameter(name)) {
      return node_ptr_->declare_parameter<std::string>(name, fallback);
    }
    const rclcpp::Parameter p = node_ptr_->get_parameter(name);
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      return p.as_string();
    }
    RCLCPP_WARN(
      node_ptr_->get_logger(),
      "Parameter '%s' is not a string (%s); using default '%s'",
      name.c_str(), p.get_type_name().c_str(), fallback.c_str());
    return fallback;
  }

  double get_double_param(const std::string & name, double fallback)
  {
    if (!node_ptr_->has_parameter(name)) {
      return node_ptr_->declare_parameter<double>(name, fallback);
    }
    const rclcpp::Parameter p = node_ptr_->get_parameter(name);
    switch (p.get_type()) {
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        return p.as_double();
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        // `twist_smooth_filter_cte: 1` in yaml is an int, and upstream's
        // get_parameter(...).as_double() would have thrown on it.
        return static_cast<double>(p.as_int());
      default:
        RCLCPP_WARN(
          node_ptr_->get_logger(),
          "Parameter '%s' is not numeric (%s); using default %f",
          name.c_str(), p.get_type_name().c_str(), fallback);
        return fallback;
    }
  }

  // ------------------------------------------------------------------------
  // subscription callback
  // ------------------------------------------------------------------------
  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    ++messages_;

    // ---- D1 -------------------------------------------------------------
    const mocap4r2_msgs::msg::RigidBody * match = nullptr;
    for (const auto & rigid_body : msg->rigidbodies) {
      if (rigid_body.rigid_body_name == rigid_body_name_) {
        match = &rigid_body;
        break;
      }
    }

    if (match == nullptr) {
      ++name_misses_;
      last_names_seen_ = join_names(*msg);
      if (miss_throttle_.ready()) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "MOCAP NAME MISMATCH: rigid body '%s' is NOT in the '%s' message. "
          "Bodies actually present: [%s]. Message DROPPED -- no pose, no transform, no twist "
          "published (upstream mocap_pose would have published the ORIGIN here). "
          "Misses so far: %lu of %lu messages. Fix `rigid_body_name` or the name in Motive; "
          "they must match exactly, including case and quoting.",
          rigid_body_name_.c_str(), mocap_topic_.c_str(), last_names_seen_.c_str(),
          static_cast<unsigned long>(name_misses_), static_cast<unsigned long>(messages_));
      }
      return;  // <<-- THE FIX. Never synthesise a pose.
    }

    // ---- D4 -------------------------------------------------------------
    double quat_norm = 0.0;
    const SampleStatus status = validate_pose(match->pose, quaternion_tolerance_, quat_norm);
    if (status != SampleStatus::OK) {
      if (status == SampleStatus::NON_FINITE) {
        ++rejects_non_finite_;
      } else {
        ++rejects_quaternion_;
      }
      if (reject_throttle_.ready()) {
        RCLCPP_ERROR(
          node_ptr_->get_logger(),
          "MOCAP SAMPLE REJECTED for '%s': %s (pos %.4f %.4f %.4f, quat %.6f %.6f %.6f %.6f, "
          "|q| %.6f, tolerance %.1e). Sample DROPPED and NOT replaced by the last good pose, so "
          "it ages out like no data. Rejects so far: %lu non-finite, %lu non-unit-quaternion.",
          rigid_body_name_.c_str(),
          status == SampleStatus::NON_FINITE ? "non-finite value" : "non-unit quaternion",
          match->pose.position.x, match->pose.position.y, match->pose.position.z,
          match->pose.orientation.x, match->pose.orientation.y,
          match->pose.orientation.z, match->pose.orientation.w,
          quat_norm, quaternion_tolerance_,
          static_cast<unsigned long>(rejects_non_finite_),
          static_cast<unsigned long>(rejects_quaternion_));
      }
      return;  // dropped, no fallback
    }

    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header = msg->header;
    pose_msg.pose = match->pose;

    ++accepted_;
    last_good_time_ = node_ptr_->now();
    has_last_good_ = true;

    process_mocap_pose(pose_msg);
  }

  static std::string join_names(const mocap4r2_msgs::msg::RigidBodies & msg)
  {
    if (msg.rigidbodies.empty()) {
      return "<empty array>";
    }
    std::ostringstream out;
    for (size_t i = 0; i < msg.rigidbodies.size(); ++i) {
      if (i != 0) {
        out << ", ";
      }
      out << "'" << msg.rigidbodies[i].rigid_body_name << "'";
    }
    return out.str();
  }

  // ------------------------------------------------------------------------
  // pose processing -- unchanged from upstream except that earth_to_map_ is
  // already fixed, so there is no latching branch here at all.
  // ------------------------------------------------------------------------
  void process_mocap_pose(const geometry_msgs::msg::PoseStamped & msg)
  {
    // mocap_pose could have a different frame_id; as upstream, the transform from
    // earth to base_link is published without checking the origin frame_id, and
    // the sample is taken as already-ENU with no axis conversion applied.
    odom_to_base_ =
      map_to_odom_.inverse() * earth_to_map_.inverse() *
      tf2::Transform(
      tf2::Quaternion(
        msg.pose.orientation.x, msg.pose.orientation.y,
        msg.pose.orientation.z, msg.pose.orientation.w),
      tf2::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    odom_to_base_msg.transform = tf2::toMsg(odom_to_base_);
    odom_to_base_msg.header.stamp = msg.header.stamp;
    odom_to_base_msg.header.frame_id = get_odom_frame();
    odom_to_base_msg.child_frame_id = get_base_frame();
    publish_transform(odom_to_base_msg);

    // Publish pose
    geometry_msgs::msg::PoseStamped pose_msg;
    // To avoid time divergence between mocap node and state estimator node
    // pose_msg.header.stamp = msg.header.stamp;
    pose_msg.header.stamp = node_ptr_->now();
    pose_msg.header.frame_id = get_earth_frame();
    pose_msg.pose = msg.pose;
    pose_msg.pose.orientation.x = orientation_alpha_ * msg.pose.orientation.x +
      (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.x;
    pose_msg.pose.orientation.y = orientation_alpha_ * msg.pose.orientation.y +
      (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.y;
    pose_msg.pose.orientation.z = orientation_alpha_ * msg.pose.orientation.z +
      (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.z;
    pose_msg.pose.orientation.w = orientation_alpha_ * msg.pose.orientation.w +
      (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.w;
    publish_pose(pose_msg);
    last_pose_msg_ = pose_msg;

    // Compute twist from mocap_pose
    const auto data = std::vector<tf2::Transform>{earth_to_map_, map_to_odom_, odom_to_base_};
    publish_twist(twist_from_pose(pose_msg, &data));
  }

  // ------------------------------------------------------------------------
  // health
  // ------------------------------------------------------------------------
  /**
   * @brief Publish /{ns}/mocap_health.
   *
   * Message type: std_msgs/String carrying a JSON object.
   *
   * Why not diagnostic_msgs/DiagnosticStatus, the obvious candidate:
   *   1. diagnostic_msgs IS NOT INSTALLED in this ROS 2 Humble image (there is no
   *      /opt/ros/humble/share/diagnostic_msgs). Depending on it would make this
   *      package unbuildable on the flight machines -- the exact class of
   *      surprise this fork exists to remove.
   *   2. DiagnosticStatus degenerates to a KeyValue[] of stringified numbers
   *      anyway, so it buys type safety it does not actually deliver.
   *   3. The consumers already speak JSON. preflight_check.py reads mocap
   *      messages by duck typing rather than importing message packages, and
   *      fake_mocap.py already publishes its fault timeline as JSON on
   *      /fake_mocap/status. One decoder covers both.
   *   4. `ros2 topic echo /drone0/mocap_health` is readable on a bench laptop
   *      with nothing extra built.
   *
   * Published on a TIMER, not from the subscription callback. A health topic
   * driven by incoming data goes silent exactly when the thing it monitors
   * fails, which is worthless; this one keeps reporting `tracked: false` and a
   * growing `age` right through a total mocap outage.
   */
  void publish_health()
  {
    const rclcpp::Time now = node_ptr_->now();
    double age = -1.0;
    if (has_last_good_) {
      age = (now - last_good_time_).seconds();
    }
    const bool tracked = has_last_good_ && (age >= 0.0) && (age <= mocap_timeout_);

    std::ostringstream json;
    json << std::fixed;
    json << "{";
    json << "\"plugin\":\"mocap_pose_guarded\"";
    json << ",\"stamp\":" << std::setprecision(6) << now.seconds();
    json << ",\"rigid_body_name\":\"" << json_escape(rigid_body_name_) << "\"";
    json << ",\"mocap_topic\":\"" << json_escape(mocap_topic_) << "\"";
    json << ",\"mocap_qos_reliability\":\"" << json_escape(mocap_qos_reliability_) << "\"";
    json << ",\"tracked\":" << (tracked ? "true" : "false");
    json << ",\"age\":" << std::setprecision(4) << age;
    json << ",\"timeout\":" << std::setprecision(4) << mocap_timeout_;
    json << ",\"messages\":" << messages_;
    json << ",\"accepted\":" << accepted_;
    json << ",\"name_misses\":" << name_misses_;
    json << ",\"rejects\":" << (rejects_non_finite_ + rejects_quaternion_);
    json << ",\"rejects_non_finite\":" << rejects_non_finite_;
    json << ",\"rejects_quaternion\":" << rejects_quaternion_;
    json << ",\"names_seen\":\"" << json_escape(last_names_seen_) << "\"";
    json << ",\"earth_to_map\":{"
         << "\"x\":" << std::setprecision(4) << earth_to_map_x_
         << ",\"y\":" << earth_to_map_y_
         << ",\"z\":" << earth_to_map_z_
         << ",\"yaw\":" << earth_to_map_yaw_ << "}";
    json << ",\"frames\":{"
         << "\"earth\":\"" << json_escape(get_earth_frame()) << "\""
         << ",\"map\":\"" << json_escape(get_map_frame()) << "\""
         << ",\"odom\":\"" << json_escape(get_odom_frame()) << "\""
         << ",\"base\":\"" << json_escape(get_base_frame()) << "\"}";
    json << "}";

    std_msgs::msg::String out;
    out.data = json.str();
    health_pub_->publish(out);
  }

  static std::string json_escape(const std::string & in)
  {
    std::ostringstream out;
    for (const char c : in) {
      switch (c) {
        case '"':
          out << "\\\"";
          break;
        case '\\':
          out << "\\\\";
          break;
        case '\n':
          out << "\\n";
          break;
        case '\r':
          out << "\\r";
          break;
        case '\t':
          out << "\\t";
          break;
        default:
          if (static_cast<unsigned char>(c) < 0x20) {
            out << "\\u00";
          } else {
            out << c;
          }
      }
    }
    return out.str();
  }
};

}  // namespace mocap_pose_guarded

#endif  // AS2_MOCAP_GUARDED__MOCAP_POSE_GUARDED_HPP_
