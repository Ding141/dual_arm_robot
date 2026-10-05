// Isolated DDS fixture: real controller + mock interfaces, no hardware plugin.
#include <chrono>
#include <cstdlib>
#include <string>
#include <vector>
#include "ieir_controllers/joint_position_controller.hpp"

int main(int argc, char ** argv)
{
  const std::string side = argc > 1 ? argv[1] : "left";
  const double seconds = argc > 2 ? std::stod(argv[2]) : 10.0;
  if ((side != "left" && side != "right") || seconds <= 0.0) {return 2;}
  rclcpp::init(0, nullptr);
  auto controller = std::make_shared<ieir_controllers::JointPositionController>();
  if (controller->init("joint_position_controller") != controller_interface::return_type::OK) {
    return 3;
  }
  std::vector<std::string> names;
  for (size_t i = 0; i < 7; ++i) {names.push_back(side + "_joint_" + std::to_string(i));}
  controller->get_node()->set_parameters({rclcpp::Parameter("joints", names),
    rclcpp::Parameter("kp_gains", std::vector<double>(7, 12.0)),
    rclcpp::Parameter("kd_gains", std::vector<double>(7, 0.5))});
  if (controller->configure().label() != "inactive") {return 4;}
  std::vector<double> q(7), v(7, 0.0), commands(28, 0.0);
  std::vector<hardware_interface::CommandInterface> cmd;
  std::vector<hardware_interface::StateInterface> state;
  cmd.reserve(28); state.reserve(14);
  const std::vector<std::string> fields = {"position", "velocity", "stiffness", "damping"};
  for (size_t i = 0; i < 7; ++i) {
    q[i] = 0.01 * i;
    for (size_t j = 0; j < 4; ++j) {cmd.emplace_back(names[i], fields[j], &commands[i * 4 + j]);}
    state.emplace_back(names[i], "position", &q[i]);
    state.emplace_back(names[i], "velocity", &v[i]);
  }
  std::vector<hardware_interface::LoanedCommandInterface> loaned_cmd;
  std::vector<hardware_interface::LoanedStateInterface> loaned_state;
  for (auto & x : cmd) {loaned_cmd.emplace_back(x);}
  for (auto & x : state) {loaned_state.emplace_back(x);}
  controller->assign_interfaces(std::move(loaned_cmd), std::move(loaned_state));
  if (controller->get_node()->activate().label() != "active") {return 5;}
  auto node = std::make_shared<rclcpp::Node>("w3_controller_mock_state");
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
  sensor_msgs::msg::JointState msg;
  msg.name = names; msg.position = q; msg.velocity = v; msg.effort.assign(7, 1.0);
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  rclcpp::WallRate rate(300.0);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    const auto now = node->now();
    controller->update(now, rclcpp::Duration::from_seconds(1.0 / 300.0));
    msg.header.stamp = now;
    pub->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(controller->get_node()->get_node_base_interface());
    rate.sleep();
  }
  controller->get_node()->deactivate();
  rclcpp::shutdown();
  return 0;
}
