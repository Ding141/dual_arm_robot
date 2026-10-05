#include <chrono>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "usb2can/serial_port.hpp"
#include "usb2can/protocol.hpp"

#include "usb2can/msg/dcu_command.hpp"
#include "usb2can/msg/dcu_feedback.hpp"

using namespace std::chrono_literals;

namespace usb2can {

class Usb2CanNode : public rclcpp::Node {
public:
  Usb2CanNode() : rclcpp::Node("usb2can_node") {
    // ----- parameters -----
    device_   = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_     = declare_parameter<int>("baudrate", 921600);
    header0_  = static_cast<uint8_t>(declare_parameter<int>("header0", 0xAA));
    header1_  = static_cast<uint8_t>(declare_parameter<int>("header1", 0x55));
    frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    publish_feedback_ = declare_parameter<bool>("publish_feedback", true);

    // ----- publishers -----
    auto qos = rclcpp::SensorDataQoS().keep_last(10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
    if (publish_feedback_) {
      fb_pub_ = create_publisher<usb2can::msg::DcuFeedback>("dcu/feedback", qos);
    }

    // ----- subscriber -----
    cmd_sub_ = create_subscription<usb2can::msg::DcuCommand>(
        "dcu/command", rclcpp::QoS(10),
        std::bind(&Usb2CanNode::on_command, this, std::placeholders::_1));

    // ----- serial -----
    serial_.set_frame_callback([this](const uint8_t* buf, size_t len) {
      this->on_frame(buf, len);
    });

    // First connection attempt is non-fatal; the watchdog will keep retrying.
    try_connect();

    // Watchdog: every 2 s, if the serial link is down, try to (re)open it.
    reconnect_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        [this]() {
          if (!serial_.is_connected()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Serial '%s' disconnected, reconnecting...",
                                 device_.c_str());
            try_connect();
          }
        });
  }

  ~Usb2CanNode() override { serial_.close(); }

private:
  void try_connect() {
    std::string err;
    // serial_.open() also tears down any previous fd / rx-thread internally.
    if (serial_.open(device_, baud_, header0_, header1_, FRAME_SIZE, err)) {
      RCLCPP_INFO(get_logger(), "Opened %s @ %d baud (low-latency)",
                  device_.c_str(), baud_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot open %s: %s (will retry every 2s)",
                           device_.c_str(), err.c_str());
    }
  }

  void on_command(const usb2can::msg::DcuCommand::SharedPtr msg) {
    DownlinkFrame f;
    f.ctrl1.cmd = msg->ctrl1.cmd;
    std::memcpy(f.ctrl1.payload.data(), msg->ctrl1.payload.data(), 64);
    f.ctrl2.cmd = msg->ctrl2.cmd;
    std::memcpy(f.ctrl2.payload.data(), msg->ctrl2.payload.data(), 64);
    f.ctrl3.cmd = msg->ctrl3.cmd;
    std::memcpy(f.ctrl3.payload.data(), msg->ctrl3.payload.data(), 64);
    f.imu_cmd   = msg->imu_cmd;

    uint8_t buf[FRAME_SIZE];
    pack_downlink(f, header0_, header1_, buf);

    if (!serial_.write_frame(buf, FRAME_SIZE)) {
      RCLCPP_WARN(get_logger(), "Serial write failed");
    }
  }

  void on_frame(const uint8_t* buf, size_t len) {
    if (len != FRAME_SIZE) return;
    UplinkFrame uf;
    if (!parse_uplink(buf, header0_, header1_, uf)) {
      // CRC or header mismatch. Counting could be added.
      return;
    }

    auto stamp = now();

    // IMU --------------------------------------------------------------
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = frame_id_;
    imu_msg.linear_acceleration.x = uf.imu.acc_x;
    imu_msg.linear_acceleration.y = uf.imu.acc_y;
    imu_msg.linear_acceleration.z = uf.imu.acc_z;
    imu_msg.angular_velocity.x = uf.imu.gyr_x;
    imu_msg.angular_velocity.y = uf.imu.gyr_y;
    imu_msg.angular_velocity.z = uf.imu.gyr_z;
    imu_msg.orientation.w = uf.imu.qw;
    imu_msg.orientation.x = uf.imu.qx;
    imu_msg.orientation.y = uf.imu.qy;
    imu_msg.orientation.z = uf.imu.qz;
    // Unknown covariance -> mark with -1 in [0] per ROS convention.
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu_msg);

    // Feedback ---------------------------------------------------------
    if (publish_feedback_ && fb_pub_) {
      usb2can::msg::DcuFeedback fb;
      fb.header.stamp = stamp;
      fb.header.frame_id = frame_id_;
      std::memcpy(fb.ctrl1.data(), uf.ctrl1.data(), 64);
      std::memcpy(fb.ctrl2.data(), uf.ctrl2.data(), 64);
      std::memcpy(fb.ctrl3.data(), uf.ctrl3.data(), 64);
      fb.imu = imu_msg;
      fb_pub_->publish(fb);
    }
  }

  // params
  std::string device_;
  int  baud_ = 921600;
  uint8_t header0_ = 0xAA;
  uint8_t header1_ = 0x55;
  std::string frame_id_ = "imu_link";
  bool publish_feedback_ = true;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr     imu_pub_;
  rclcpp::Publisher<usb2can::msg::DcuFeedback>::SharedPtr fb_pub_;
  rclcpp::Subscription<usb2can::msg::DcuCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  SerialPort serial_;
};

}  // namespace usb2can

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<usb2can::Usb2CanNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "usb2can_node fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
