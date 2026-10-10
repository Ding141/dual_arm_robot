#include "ieir_controllers/joint_position_controller.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <unordered_set>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ieir_controllers
{

namespace
{
// builtin_interfaces::Duration -> seconds (as double).
inline double duration_to_sec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
}
}  // namespace

JointPositionController::JointPositionController() = default;

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // IMPORTANT: keep this layout in sync with kCmdStride in the header.
  // For joint i, the command interface indices are:
  //   4*i + 0: position
  //   4*i + 1: velocity
  //   4*i + 2: stiffness  (== motor kp)
  //   4*i + 3: damping    (== motor kd)
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    cfg.names.push_back(name + "/stiffness");
    cfg.names.push_back(name + "/damping");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // For joint i, the state interface indices are:
  //   2*i + 0: position
  //   2*i + 1: velocity
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return cfg;
}

controller_interface::CallbackReturn JointPositionController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("kp_gains", {});
    auto_declare<std::vector<double>>("kd_gains", {});
    auto_declare<std::string>("command_topic", "/joint_position_command");
    auto_declare<std::vector<double>>("max_velocity", {});
    auto_declare<std::vector<double>>("max_acceleration", {});
    auto_declare<std::vector<double>>("max_jerk", {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointPositionController on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  active_ = false;

  joint_names_ = node->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  kp_gains_ = node->get_parameter("kp_gains").as_double_array();
  kd_gains_ = node->get_parameter("kd_gains").as_double_array();
  if (kp_gains_.size() != joint_names_.size() ||
      kd_gains_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(node->get_logger(),
                 "kp_gains (%zu) and kd_gains (%zu) must both have size == joints (%zu)",
                 kp_gains_.size(), kd_gains_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  // Reject negatives -- DM expects non-negative kp/kd.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (!std::isfinite(kp_gains_[i]) || !std::isfinite(kd_gains_[i]) || kp_gains_[i] < 0.0 || kd_gains_[i] < 0.0) {
      RCLCPP_ERROR(node->get_logger(),
                   "Joint '%s': negative gain (kp=%.3f, kd=%.3f) is not allowed",
                   joint_names_[i].c_str(), kp_gains_[i], kd_gains_[i]);
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  command_topic_ = node->get_parameter("command_topic").as_string();
  auto read_limits = [&](const char *name, double fallback, std::vector<double> &out) {
    out=node->get_parameter(name).as_double_array();
    if(out.empty()) out.assign(joint_names_.size(),fallback);
    return out.size()==joint_names_.size() && std::all_of(out.begin(),out.end(),
      [](double v) {return std::isfinite(v) && v>0;});
  };
  if(!read_limits("max_velocity",.4,max_velocity_) ||
     !read_limits("max_acceleration",1.75,max_acceleration_) ||
     !read_limits("max_jerk",3.5,max_jerk_)) {
    RCLCPP_ERROR(node->get_logger(),"Motion limits must be positive finite per-joint arrays");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_state_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
    "~/command_state", rclcpp::SensorDataQoS());
  command_state_rt_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      command_state_pub_);
  // No subscriber/update can see this message before activation.
  command_state_rt_pub_->lock();
  auto & state = command_state_rt_pub_->msg_;
  state.name = joint_names_;
  state.position.resize(joint_names_.size());
  state.velocity.resize(joint_names_.size());
  state.effort.clear();
  command_state_rt_pub_->unlock();

  // Subscribe in non-RT thread; hand off to update() via realtime buffer.
  // The buffer holds at most one trajectory; new ones replace the previous
  // (queue depth 1 effectively), which matches the "latest goal wins"
  // semantics typical of impedance control.
  traj_sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      if(!valid_message(*msg)) {
        RCLCPP_WARN(get_node()->get_logger(),"Rejected malformed JointTrajectory");
        return;
      }
      // Defensive copy so the subscriber-thread shared_ptr cannot be
      // mutated under the RT thread's feet.
      auto copy = std::make_shared<trajectory_msgs::msg::JointTrajectory>(*msg);
      traj_buffer_.writeFromNonRT(copy);
    });

  RCLCPP_INFO(node->get_logger(),
              "JointPositionController configured with %zu joint(s); "
              "subscribing to '%s' (trajectory_msgs/JointTrajectory)",
              joint_names_.size(), command_topic_.c_str());
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(node->get_logger(),
                "  %-14s  kp=%6.2f  kd=%6.3f",
                joint_names_[i].c_str(), kp_gains_[i], kd_gains_[i]);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  const size_t n = joint_names_.size();

  // Sanity: ros2_control must have laid out interfaces in the same order
  // we requested in command_interface_configuration() / state_interface_configuration().
  if (command_interfaces_.size() != n * kCmdStride) {
    RCLCPP_ERROR(node->get_logger(),
                 "Expected %zu command interfaces, got %zu",
                 n * kCmdStride, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_interfaces_.size() != n * kStateStride) {
    RCLCPP_ERROR(node->get_logger(),
                 "Expected %zu state interfaces, got %zu",
                 n * kStateStride, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Bumpless start: hold setpoint = current measured position. The first
  // update() will publish (cmd_pos, cmd_vel) = (current pos, 0) with the
  // yaml kp/kd, so the motor PD computes ~zero correction and the robot
  // does not jump even though kp just went from 0 to a non-zero value.
  hold_pos_.assign(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    hold_pos_[i] = state_interfaces_[i * kStateStride + 0].get_value();
  }
  // No trajectory has been picked up yet, so the pre-roll snapshot is
  // simply the activation-time pose. It will be overwritten the moment
  // try_pickup_new_trajectory() accepts a trajectory.
  pre_roll_start_pos_ = hold_pos_;
  hold_vel_.assign(n,0.); hold_acc_.assign(n,0.);
  segments_.assign(n,{});
  pending_traj_.reset(); consumed_traj_.reset();

  // Drop any trajectory that arrived while we were inactive -- those
  // setpoints are stale and starting from them would defeat the bumpless
  // semantics above.
  traj_buffer_.reset();
  active_traj_.reset();
  traj_joint_idx_.clear();

  active_ = true;
  RCLCPP_INFO(node->get_logger(),
              "JointPositionController activated; holding current position. "
              "Send a trajectory_msgs/JointTrajectory on '%s' to move.",
              command_topic_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPositionController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  active_ = false;
  // Hand control back gracefully: zero the gains and the velocity command.
  // With kp == kd == 0 the dm_hardware_interface's bumpless mirror kicks
  // in and tracks the live motor position, so even if no other controller
  // takes over the motor cannot snap toward a stale target.
  // (cmd_pos is overwritten by that mirror too, but we set it to current
  // measured position here to be explicit and self-documenting.)
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    const double q = state_interfaces_[i * kStateStride + 0].get_value();
    command_interfaces_[i * kCmdStride + 0].set_value(q);     // position
    command_interfaces_[i * kCmdStride + 1].set_value(0.0);   // velocity
    command_interfaces_[i * kCmdStride + 2].set_value(0.0);   // stiffness (kp)
    command_interfaces_[i * kCmdStride + 3].set_value(0.0);   // damping   (kd)
  }
  active_traj_.reset();
  pending_traj_.reset(); segments_.clear();
  traj_joint_idx_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool JointPositionController::valid_message(const trajectory_msgs::msg::JointTrajectory &msg) const
{
  if(msg.points.empty() || msg.joint_names.empty()) return false;
  std::unordered_set<std::string> names;
  for(const auto &name:msg.joint_names) {
    if(!names.insert(name).second ||
       std::find(joint_names_.begin(),joint_names_.end(),name)==joint_names_.end()) return false;
  }
  double last=-1;
  const auto n=msg.joint_names.size();
  for(const auto &p:msg.points) {
    double t=duration_to_sec(p.time_from_start);
    if(!std::isfinite(t) || t<0 || t<=last || p.positions.size()!=n ||
       (!p.velocities.empty() && p.velocities.size()!=n) ||
       (!p.accelerations.empty() && (p.accelerations.size()!=n || p.velocities.empty()))) return false;
    for(const auto *values:{&p.positions,&p.velocities,&p.accelerations})
      if(!std::all_of(values->begin(),values->end(),[](double x){return std::isfinite(x);})) return false;
    last=t;
  }
  return true;
}

TrajectorySpline JointPositionController::stopping_segment(
  size_t i,double time,double q,double v,double a) const
{
  double duration=std::max({.1,4*std::abs(v)/max_acceleration_[i],
    4*std::abs(a)/max_jerk_[i],std::sqrt(12*std::abs(v)/max_jerk_[i])});
  for(int retry=0;retry<30;++retry) {
    const double goal=q+v*duration/2+a*duration*duration/12;
    auto s=TrajectorySpline::make(time,time+duration,q,v,a,goal,0,0);
    if(s.bounded(max_velocity_[i],max_acceleration_[i],max_jerk_[i])) return s;
    duration*=1.25;
  }
  // Should only be reachable for an invalid outgoing state; keep diagnostics explicit.
  throw std::runtime_error("Cannot construct a bounded stopping segment");
}

bool JointPositionController::install_trajectory(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> &msg,
  const rclcpp::Time &start,const rclcpp::Time &now)
{
  const double elapsed=std::max(0.,(now-start).seconds());
  const auto &pts=msg->points;
  if(duration_to_sec(pts.back().time_from_start)<=elapsed) return false;
  std::vector<std::vector<TrajectorySpline>> next(joint_names_.size());
  std::vector<size_t> mapping(joint_names_.size(),SIZE_MAX_LOCAL);
  std::vector<double> anchor(joint_names_.size());
  for(size_t i=0;i<joint_names_.size();++i) {
    double q,v,a; sample_setpoint(i,now,q,v,a); anchor[i]=q;
    auto it=std::find(msg->joint_names.begin(),msg->joint_names.end(),joint_names_[i]);
    if(it==msg->joint_names.end()) {
      next[i].push_back(stopping_segment(i,elapsed,q,v,a)); continue;
    }
    const size_t k=std::distance(msg->joint_names.begin(),it); mapping[i]=k;
    auto velocity=[&](size_t p) {
      if(!pts[p].velocities.empty()) return pts[p].velocities[k];
      if(p==0 || p+1==pts.size()) return 0.;
      const double left=(pts[p].positions[k]-pts[p-1].positions[k]) /
        (duration_to_sec(pts[p].time_from_start)-duration_to_sec(pts[p-1].time_from_start));
      const double right=(pts[p+1].positions[k]-pts[p].positions[k]) /
        (duration_to_sec(pts[p+1].time_from_start)-duration_to_sec(pts[p].time_from_start));
      return left*right<=0?0.:2*left*right/(left+right);
    };
    double previous=elapsed;
    for(size_t p=0;p<pts.size();++p) {
      const double t=duration_to_sec(pts[p].time_from_start);
      // Avoid numerically ill-conditioned microsecond handoff splines at a knot.
      if(t<=elapsed+1e-8 || (t<elapsed+.02 && p+1<pts.size())) continue;
      const double vn=velocity(p), an=pts[p].accelerations.empty()?0.:pts[p].accelerations[k];
      auto seg=TrajectorySpline::make(previous,t,q,v,a,pts[p].positions[k],vn,an);
      if(!seg.bounded(max_velocity_[i],max_acceleration_[i],max_jerk_[i])) {
        RCLCPP_WARN(get_node()->get_logger(),"Rejected trajectory: %s segment %.6f..%.6f exceeds v/a/jerk limits",
          joint_names_[i].c_str(),previous,t); return false;
      }
      next[i].push_back(seg); previous=t; q=pts[p].positions[k];v=vn;a=an;
    }
    next[i].push_back(stopping_segment(i,previous,q,v,a));
  }
  segments_=std::move(next); traj_joint_idx_=std::move(mapping);
  pre_roll_start_pos_=std::move(anchor);active_traj_=msg;active_traj_start_=start;
  return true;
}

void JointPositionController::try_pickup_new_trajectory(const rclcpp::Time &now)
{
  auto *latest=traj_buffer_.readFromRT();
  // A monotonically changing pointer avoids clearing a newer non-RT write from update().
  if(latest && *latest && *latest!=consumed_traj_) {
    consumed_traj_=*latest;
    if(valid_message(*consumed_traj_)) {
      auto stamp=rclcpp::Time(consumed_traj_->header.stamp,now.get_clock_type());
      if(stamp.nanoseconds()==0) stamp=now;
      pending_traj_=consumed_traj_; pending_start_=stamp;
    }
  }
  if(pending_traj_ && now>=pending_start_) {
    auto msg=pending_traj_; pending_traj_.reset();
    try {install_trajectory(msg,pending_start_,now);}
    catch(const std::exception &e) {
      RCLCPP_WARN(get_node()->get_logger(),"Rejected trajectory: %s",e.what());
    }
  }
}

void JointPositionController::sample_setpoint(
  size_t i,const rclcpp::Time &now,double &q,double &v,double &a) const
{
  q=hold_pos_[i];v=0.;a=0.;
  if(!active_traj_ || i>=segments_.size() || segments_[i].empty()) return;
  const double elapsed=(now-active_traj_start_).seconds();
  const auto &segments=segments_[i];
  if(elapsed<segments.front().start) {
    segments.front().sample(segments.front().start,q,v,a); return;
  }
  for(const auto &seg:segments) {
    if(elapsed<seg.end) {seg.sample(elapsed,q,v,a);return;}
  }
  segments.back().sample(segments.back().end,q,v,a);v=0.;a=0.;
}

controller_interface::return_type JointPositionController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!active_) {
    return controller_interface::return_type::OK;
  }
  try_pickup_new_trajectory(time);

  const bool publish_state = command_state_rt_pub_ && command_state_rt_pub_->trylock();
  if (publish_state) {
    command_state_rt_pub_->msg_.header.stamp = time;
  }
  const size_t n = joint_names_.size();
  for (size_t i = 0; i < n; ++i) {
    double pos_des, vel_des, acc_des;
    sample_setpoint(i, time, pos_des, vel_des, acc_des);

    // Update the hold so that:
    //   (a) when the trajectory ends, hold_pos_ already equals the final
    //       setpoint, so the controller naturally stays put;
    //   (b) on the next on_activate transition (after deactivate), the
    //       starting position is the most recent commanded one rather
    //       than a measurement that may drift under load.
    hold_pos_[i] = pos_des;
    hold_vel_[i] = vel_des; hold_acc_[i] = acc_des;

    command_interfaces_[i * kCmdStride + 0].set_value(pos_des);
    command_interfaces_[i * kCmdStride + 1].set_value(vel_des);
    command_interfaces_[i * kCmdStride + 2].set_value(kp_gains_[i]);
    command_interfaces_[i * kCmdStride + 3].set_value(kd_gains_[i]);
    if (publish_state) {
      command_state_rt_pub_->msg_.position[i] = pos_des;
      command_state_rt_pub_->msg_.velocity[i] = vel_des;
    }
  }
  if (publish_state) {
    command_state_rt_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

}  // namespace ieir_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ieir_controllers::JointPositionController,
  controller_interface::ControllerInterface)
