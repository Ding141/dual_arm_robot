#include <gtest/gtest.h>
#include <fstream>
#include <thread>
#include <chrono>
#include "ieir_controllers/gravity_compensation_controller.hpp"
using namespace std::chrono_literals;
class FrictionReferenceTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};
TEST_F(FrictionReferenceTest, FreshReferenceMovesFeedforwardAtZeroMeasuredSpeedAndStaleDecays) {
  const std::string model_path="/tmp/ieir_friction_reference_test_"+
    std::to_string(::getpid())+".yaml";
  {std::ofstream f(model_path);f<<"TEST: {viscous: 0.0, coulomb_pos: 0.45, coulomb_neg: 0.45}\n";}
  ieir_controllers::GravityCompensationController c;
  ASSERT_EQ(c.init("friction_reference_test"),controller_interface::return_type::OK);
  c.get_node()->set_parameters({
    rclcpp::Parameter("joints",std::vector<std::string>{"right_joint_0"}),
    rclcpp::Parameter("robot_description",std::string(
      "<robot name='test'><link name='base'/><link name='child'>"
      "<inertial><mass value='1'/><inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial>"
      "</link><joint name='right_joint_0' type='revolute'><parent link='base'/>"
      "<child link='child'/><axis xyz='0 0 1'/><limit effort='7' lower='-1' upper='1' velocity='1'/>"
      "</joint></robot>")),
    rclcpp::Parameter("motor_types",std::vector<std::string>{"TEST"}),
    rclcpp::Parameter("friction_model_yaml",model_path),
    rclcpp::Parameter("friction_compensation_enabled",true),
    rclcpp::Parameter("smooth_friction_enabled",true),
    rclcpp::Parameter("friction_gains",std::vector<double>{.8}),
    rclcpp::Parameter("friction_reference_topic",std::string("/offline_friction_reference"))});
  ASSERT_EQ(c.configure().label(),"inactive");
  double q=0,v=0,effort=0;
  hardware_interface::CommandInterface effort_if("right_joint_0","effort",&effort);
  hardware_interface::StateInterface q_if("right_joint_0","position",&q),v_if("right_joint_0","velocity",&v);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(effort_if);states.emplace_back(q_if);states.emplace_back(v_if);
  c.assign_interfaces(std::move(commands),std::move(states));
  ASSERT_EQ(c.get_node()->activate().label(),"active");
  auto sender=std::make_shared<rclcpp::Node>("friction_reference_sender");
  auto pub=sender->create_publisher<sensor_msgs::msg::JointState>("/offline_friction_reference",rclcpp::SensorDataQoS());
  for(int i=0;i<100 && pub->get_subscription_count()==0;i++) {rclcpp::spin_some(c.get_node()->get_node_base_interface());std::this_thread::sleep_for(5ms);}
  ASSERT_EQ(pub->get_subscription_count(),1u);
  const auto now=c.get_node()->now();auto dt=rclcpp::Duration::from_seconds(1./300);
  c.update(now,dt);EXPECT_DOUBLE_EQ(effort,0);
  sensor_msgs::msg::JointState msg;msg.header.stamp=now;msg.name={"right_joint_0"};msg.velocity={.1};
  for(int i=0;i<10;i++) {pub->publish(msg);std::this_thread::sleep_for(2ms);rclcpp::spin_some(c.get_node()->get_node_base_interface());}
  for(int i=0;i<14;i++) {c.update(now+rclcpp::Duration::from_seconds(i/300.),dt);EXPECT_LE(std::abs(effort),.361);}
  EXPECT_NEAR(effort,.36,1e-8);EXPECT_DOUBLE_EQ(v,0);
  c.update(now+rclcpp::Duration::from_seconds(.1),dt);EXPECT_NEAR(effort,.36-8./300,1e-8);
  for(int i=0;i<30;i++) {c.update(now+rclcpp::Duration::from_seconds(.2+i/300.),dt);}
  EXPECT_DOUBLE_EQ(effort,0);
  // Future stamps and duplicate joint names must never produce friction.
  msg.header.stamp=now+rclcpp::Duration::from_seconds(10);msg.name={"right_joint_0","right_joint_0"};msg.velocity={1.,1.};
  for(int i=0;i<5;i++) {pub->publish(msg);std::this_thread::sleep_for(2ms);rclcpp::spin_some(c.get_node()->get_node_base_interface());}
  c.update(now+rclcpp::Duration::from_seconds(.4),dt);EXPECT_DOUBLE_EQ(effort,0);
  c.get_node()->deactivate();EXPECT_DOUBLE_EQ(effort,0);std::remove(model_path.c_str());
}
