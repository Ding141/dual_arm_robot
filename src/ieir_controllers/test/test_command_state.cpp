// Offline tests of the real controller with double-backed loaned interfaces.
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "ieir_controllers/joint_position_controller.hpp"

namespace ieir_controllers
{
class JointPositionControllerTestPeer
{
public:
  static bool pending(JointPositionController & c)
  {
    auto p = c.traj_buffer_.readFromRT();
    return p && *p;
  }
  static void lock(JointPositionController & c) {c.command_state_rt_pub_->lock();}
  static void unlock(JointPositionController & c) {c.command_state_rt_pub_->unlock();}
};
}
using Controller = ieir_controllers::JointPositionController;
using Peer = ieir_controllers::JointPositionControllerTestPeer;
using JS = sensor_msgs::msg::JointState;
using Traj = trajectory_msgs::msg::JointTrajectory;
using namespace std::chrono_literals;

class CommandStateTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {setup(7);}
  void TearDown() override
  {
    if (controller && controller->get_state().label() == "active") {
      controller->get_node()->deactivate();
    }
    sub.reset(); sender.reset(); observer.reset(); controller.reset();
  }
  void setup(size_t n)
  {
    TearDown();
    names.clear();
    for (size_t i = 0; i < n; ++i) {
      names.push_back((n == 14 && i < 7 ? "left_joint_" : "right_joint_") +
        std::to_string(i % 7));
    }
    controller = std::make_unique<Controller>();
    ASSERT_EQ(controller->init("joint_position_controller"), controller_interface::return_type::OK);
    controller->get_node()->set_parameters({
      rclcpp::Parameter("joints", names),
      rclcpp::Parameter("kp_gains", std::vector<double>(n, 12.0)),
      rclcpp::Parameter("kd_gains", std::vector<double>(n, 0.5))});
    ASSERT_EQ(controller->configure().label(), "inactive");
    command_values.assign(n * 4, -999.0);
    state_values.assign(n * 2, 0.0);
    commands.clear(); states.clear();
    commands.reserve(n * 4); states.reserve(n * 2);
    const std::vector<std::string> fields = {"position", "velocity", "stiffness", "damping"};
    for (size_t i = 0; i < n; ++i) {
      state_values[i * 2] = 0.01 * i;
      for (size_t k = 0; k < 4; ++k) {
        commands.emplace_back(names[i], fields[k], &command_values[i * 4 + k]);
      }
      for (size_t k = 0; k < 2; ++k) {
        states.emplace_back(names[i], fields[k], &state_values[i * 2 + k]);
      }
    }
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (auto & interface : commands) {loaned_commands.emplace_back(interface);}
    for (auto & interface : states) {loaned_states.emplace_back(interface);}
    controller->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    observer = std::make_shared<rclcpp::Node>("command_state_test_observer");
    sub = observer->create_subscription<JS>("/joint_position_controller/command_state",
      rclcpp::SensorDataQoS(), [this](JS::ConstSharedPtr msg) {received.push_back(*msg);});
    sender = observer->create_publisher<Traj>("/joint_position_command", rclcpp::SystemDefaultsQoS());
    ASSERT_TRUE(wait([this]() {
      return sender->get_subscription_count() == 1 &&
        observer->count_publishers("/joint_position_controller/command_state") == 1;
    }));
    received.clear();
    ASSERT_EQ(controller->get_node()->activate().label(), "active");
  }
  template<typename Predicate> bool wait(Predicate done)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    do {
      rclcpp::spin_some(observer);
      rclcpp::spin_some(controller->get_node()->get_node_base_interface());
      if (done()) {return true;}
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }
  void trajectory(const std::vector<std::string> & joint_names,
    const std::vector<std::vector<double>> & positions,
    const std::vector<double> & seconds, int64_t start_ns = 0)
  {
    Traj msg;
    msg.joint_names = joint_names;
    msg.header.stamp = rclcpp::Time(start_ns);
    for (size_t k = 0; k < positions.size(); ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      p.positions = positions[k];
      p.time_from_start = rclcpp::Duration::from_seconds(seconds[k]);
      msg.points.push_back(p);
    }
    sender->publish(msg);
    ASSERT_TRUE(wait([this]() {return Peer::pending(*controller);}));
  }
  void step(int64_t stamp_ns)
  {
    // A realtime publisher is allowed to drop a tick. Repeat the same manually
    // selected sample time until the worker is available; no interpolation in tests.
    ASSERT_TRUE(wait([this, stamp_ns]() {
      EXPECT_EQ(controller->update(rclcpp::Time(stamp_ns, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.003)),
        controller_interface::return_type::OK);
      for (const auto & msg : received) {
        if (rclcpp::Time(msg.header.stamp).nanoseconds() == stamp_ns) {return true;}
      }
      return false;
    }));
    const JS * selected = nullptr;
    for (const auto & msg : received) {
      if (rclcpp::Time(msg.header.stamp).nanoseconds() == stamp_ns) {selected = &msg;}
    }
    ASSERT_NE(selected, nullptr);
    EXPECT_EQ(selected->name, names);
    ASSERT_EQ(selected->position.size(), names.size());
    ASSERT_EQ(selected->velocity.size(), names.size());
    EXPECT_TRUE(selected->effort.empty());
    EXPECT_EQ(rclcpp::Time(selected->header.stamp).nanoseconds(), stamp_ns);
    for (size_t i = 0; i < names.size(); ++i) {
      EXPECT_TRUE(std::isfinite(selected->position[i]));
      EXPECT_TRUE(std::isfinite(selected->velocity[i]));
      EXPECT_NEAR(selected->position[i], command_values[i * 4], 1e-9);
      EXPECT_NEAR(selected->velocity[i], command_values[i * 4 + 1], 1e-9);
      EXPECT_DOUBLE_EQ(command_values[i * 4 + 2], 12.0);
      EXPECT_DOUBLE_EQ(command_values[i * 4 + 3], 0.5);
    }
  }
  void value(size_t joint, double q, double qdot)
  {
    EXPECT_NEAR(command_values[joint * 4], q, 1e-9);
    EXPECT_NEAR(command_values[joint * 4 + 1], qdot, 1e-9);
  }
  std::unique_ptr<Controller> controller;
  std::shared_ptr<rclcpp::Node> observer;
  rclcpp::Publisher<Traj>::SharedPtr sender;
  rclcpp::Subscription<JS>::SharedPtr sub;
  std::vector<std::string> names;
  std::vector<double> command_values, state_values;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  std::vector<JS> received;
};

TEST_F(CommandStateTest, CMD01HoldAndCMD10StampFieldsQoS)
{
  step(1000123456);
  for (size_t i = 0; i < names.size(); ++i) {value(i, 0.01 * i, 0.0);}
  auto infos = observer->get_publishers_info_by_topic("/joint_position_controller/command_state");
  ASSERT_EQ(infos.size(), 1u);
  EXPECT_EQ(infos[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(infos[0].qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  std::cout << "command_state endpoint: /joint_position_controller/command_state "
    << "BEST_EFFORT/VOLATILE KEEP_LAST depth=" << infos[0].qos_profile().depth() << std::endl;
}
TEST_F(CommandStateTest, CMD02LinearAndCMD05EndHold)
{
  trajectory({names[0]}, {{0.0}, {0.2}}, {0.0, 1.0});
  step(1000000000); value(0, 0.0, 0.0);
  step(1500000000); value(0, 0.1, 0.2);
  step(2000000000); value(0, 0.2, 0.0);
  step(2100000000); value(0, 0.2, 0.0);
}
TEST_F(CommandStateTest, CMD03FrozenPreRollAndCMD04Boundary)
{
  trajectory({names[0]}, {{0.2}}, {1.0});
  step(1000000000); value(0, 0.2, 0.0);  // Existing t==0 first-point behavior.
  step(1250000000); value(0, 0.05, 0.2);
  step(1500000000); value(0, 0.1, 0.2);
  step(1750000000); value(0, 0.15, 0.2);
  trajectory({names[0]}, {{0.4}}, {1.0}, 3000000000);
  step(2000000000); value(0, 0.4, 0.0);  // Existing future-start behavior.
  step(3000000000); value(0, 0.4, 0.0);
  step(3500000000); value(0, 0.275, 0.25);  // Frozen pickup pose was 0.15.
}
TEST_F(CommandStateTest, CMD06PartialReorderedAndReplacement)
{
  trajectory({names[2], names[0]}, {{0.02, 0.0}, {0.42, 0.2}}, {0.0, 1.0});
  step(1000000000); step(1500000000);
  value(0, 0.1, 0.2); value(2, 0.22, 0.4); value(1, 0.01, 0.0);
  trajectory({names[1]}, {{0.31}}, {1.0});
  step(1600000000); value(1, 0.31, 0.0);
  step(2100000000); value(1, 0.16, 0.3);
  value(0, 0.1, 0.0); value(2, 0.22, 0.0);
}
TEST_F(CommandStateTest, CMD07DualAndSingle)
{
  setup(14); step(1000000000);
  for (size_t i = 0; i < names.size(); ++i) {value(i, 0.01 * i, 0.0);}
  setup(7); step(1000000000);
}
TEST_F(CommandStateTest, CMD08DeactivateAndReactivate)
{
  step(1000000000);
  ASSERT_EQ(controller->get_node()->deactivate().label(), "inactive");
  // Drain any queued active frame before testing inactive update.
  wait([]() {return false;});
  received.clear();
  EXPECT_EQ(controller->update(rclcpp::Time(2000000000LL, RCL_ROS_TIME), rclcpp::Duration(0, 1)),
    controller_interface::return_type::OK);
  wait([]() {return false;});
  EXPECT_TRUE(received.empty());
  for (size_t i = 0; i < names.size(); ++i) {
    EXPECT_DOUBLE_EQ(command_values[i * 4 + 2], 0.0);
    EXPECT_DOUBLE_EQ(command_values[i * 4 + 3], 0.0);
    state_values[i * 2] = 0.3 + 0.01 * i;
  }
  trajectory({names[0]}, {{9.0}}, {0.0});
  ASSERT_EQ(controller->get_node()->activate().label(), "active");
  step(3000000000);
  for (size_t i = 0; i < names.size(); ++i) {value(i, 0.3 + 0.01 * i, 0.0);}
}
TEST_F(CommandStateTest, CMD09LockContentionAndSlowOrAbsentSubscriber)
{
  step(1000000000);
  Peer::lock(*controller);
  const auto begin = std::chrono::steady_clock::now();
  EXPECT_EQ(controller->update(rclcpp::Time(1500000000LL, RCL_ROS_TIME), rclcpp::Duration(0, 1)),
    controller_interface::return_type::OK);
  EXPECT_LT(std::chrono::steady_clock::now() - begin, 50ms);
  Peer::unlock(*controller);
  for (size_t i = 0; i < names.size(); ++i) {value(i, 0.01 * i, 0.0);}
  step(2000000000);
  for (const auto & msg : received) {
    EXPECT_NE(rclcpp::Time(msg.header.stamp).nanoseconds(), 1500000000);
  }
  // Do not spin the subscriber: no synchronous publish or waiting for DDS readers.
  for (int i = 0; i < 1000; ++i) {
    EXPECT_EQ(controller->update(rclcpp::Time(2000000000LL + i, RCL_ROS_TIME), rclcpp::Duration(0, 1)),
      controller_interface::return_type::OK);
  }
  sub.reset();
  EXPECT_EQ(controller->update(rclcpp::Time(3000000000LL, RCL_ROS_TIME), rclcpp::Duration(0, 1)),
    controller_interface::return_type::OK);
}
