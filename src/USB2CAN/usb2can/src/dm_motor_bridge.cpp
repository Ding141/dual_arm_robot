// DM4310 (MIT 模式) 桥接节点
//
// 在原始 /dcu/feedback /dcu/command 之上再封一层:
//   /dcu/feedback (DcuFeedback) ── unpack ──▶ /motor/ch{1,2,3}/state (MotorStateArray)
//   /motor/ch{1,2,3}/cmd (MotorCommandArray) ── pack ──▶ /dcu/command (DcuCommand)
//
// 一个进程统一处理 3 个通道，避免多 publisher 抢 /dcu/command 互相覆盖。
//
// CTRL.cmd 字节语义 (与 STM32 端约定一致):
//   0x00       该通道不转发任何 CAN 报文 (失能)
//   0xFF       广播: 整 64 字节作为 1 帧 CANFD 报文转发, CAN ID = 0
//   bit i = 1  转发 payload[i*8 .. i*8+7] 作为 1 帧 CAN 报文, CAN ID = i+1
//   ...故同一帧最多 8 个独立 8B 报文 (i ∈ [0,7], 但 0xFF 已被广播占用)
//
// 本节点对 cmd 字节的处理:
//   - 当用户向 /motor/chN/cmd 发 MotorCommandArray 时, 自动求出现电机 id 的 OR
//     作为该通道 cmd 字节; 未出现的 id 对应 bit=0, 其字节也保持上一帧或被清零.
//   - /motor/chN/enable (Bool): false 强制 cmd=0 (停止转发该通道).
//                              true  恢复使用 mask (默认行为).
//   - /motor/chN/broadcast (UInt8MultiArray[64]): 一次性发广播帧 (cmd=0xFF, 64B payload).
//
// 协议（DM 厂家 MIT 模式，每电机 8 字节，槽位即 motor id 0..7）:
//
//   TX (host -> motor):
//     B0 = pos[15:8]
//     B1 = pos[7:0]
//     B2 = vel[11:4]
//     B3 = (vel[3:0]<<4) | kp[11:8]
//     B4 = kp[7:0]
//     B5 = kd[11:4]
//     B6 = (kd[3:0]<<4) | t[11:8]
//     B7 = t[7:0]
//
//   RX (motor -> host):
//     B0 = (err<<4) | id
//     B1 = pos[15:8]
//     B2 = pos[7:0]
//     B3 = vel[11:4]
//     B4 = (vel[3:0]<<4) | t[11:8]
//     B5 = t[7:0]
//     B6 = T_mos (°C)
//     B7 = T_rotor (°C)

#include <array>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "usb2can/msg/dcu_command.hpp"
#include "usb2can/msg/dcu_feedback.hpp"
#include "usb2can/msg/motor_command_array.hpp"
#include "usb2can/msg/motor_state_array.hpp"
#include "usb2can/msg/raw_can_frame_array.hpp"
#include "usb2can/msg/motor_enable_array.hpp"

namespace usb2can {

// 12/16 位定点编解码
static inline uint16_t float_to_uint(float x, float xmin, float xmax, int bits) {
  if (x < xmin) x = xmin;
  if (x > xmax) x = xmax;
  const float span = xmax - xmin;
  const uint32_t maxv = (1u << bits) - 1u;
  return static_cast<uint16_t>((x - xmin) * static_cast<float>(maxv) / span);
}

static inline float uint_to_float(uint32_t x, float xmin, float xmax, int bits) {
  const float span = xmax - xmin;
  const uint32_t maxv = (1u << bits) - 1u;
  return static_cast<float>(x) * span / static_cast<float>(maxv) + xmin;
}

class DmMotorBridge : public rclcpp::Node {
public:
  DmMotorBridge() : Node("dm_motor_bridge") {
    // 全局默认 (DM4310 出厂值). 单个电机可在 YAML 里覆盖.
    const double g_pos = declare_parameter<double>("pos_max", 12.5);
    const double g_vel = declare_parameter<double>("vel_max", 30.0);
    const double g_tor = declare_parameter<double>("tor_max", 10.0);
    const double g_kp  = declare_parameter<double>("kp_max", 500.0);
    const double g_kd  = declare_parameter<double>("kd_max", 5.0);

    // Per-motor 限幅: 参数名 ch{1..3}.id{0..7}.{pos_max,vel_max,tor_max,kp_max,kd_max}
    // 未指定则继承全局默认.
    for (int ch = 0; ch < 3; ++ch) {
      for (int id = 0; id < 8; ++id) {
        const std::string pfx = "ch" + std::to_string(ch + 1)
                              + ".id" + std::to_string(id) + ".";
        auto& L = limits_[ch][id];
        L.pos_max = declare_parameter<double>(pfx + "pos_max", g_pos);
        L.vel_max = declare_parameter<double>(pfx + "vel_max", g_vel);
        L.tor_max = declare_parameter<double>(pfx + "tor_max", g_tor);
        L.kp_max  = declare_parameter<double>(pfx + "kp_max",  g_kp);
        L.kd_max  = declare_parameter<double>(pfx + "kd_max",  g_kd);
        L.type    = declare_parameter<std::string>(pfx + "type", "");
      }
    }

    enable_[0] = declare_parameter<bool>("enable_ch1", true);
    enable_[1] = declare_parameter<bool>("enable_ch2", true);
    enable_[2] = declare_parameter<bool>("enable_ch3", true);

    // 通道总开关默认 true (使 cmd 字节 = slot_mask)。
    // 启动时 slot_mask=0 -> cmd=0, STM32 不动, 安全。
    // 当用户开始发 /motor/chN/cmd 时, slot_mask 自动出现, STM32 按位自行使能。
    // 紧急停: 发 /motor/chN/enable false, cmd 立即变 0, STM32 看到下降沿失能。
    motor_enabled_[0] = declare_parameter<bool>("default_enable_ch1", true);
    motor_enabled_[1] = declare_parameter<bool>("default_enable_ch2", true);
    motor_enabled_[2] = declare_parameter<bool>("default_enable_ch3", true);

    imu_cmd_ = static_cast<uint8_t>(declare_parameter<int>("imu_cmd", 0));

    // Aggregation rate. Bridge previously emitted one DcuCommand per
    // /motor/chN/cmd it received, which made the outbound rate the SUM of
    // all publishers (e.g., arm hw_interface @ update_rate + gripper @
    // control_rate, on each of two channels). With multiple cooperating
    // publishers per channel that flooded the USB-CAN MCU and pushed CAN bus
    // utilization past comfortable limits.
    //
    // New semantics:
    //   - on_cmd(...) MERGES the incoming msg into the per-channel cache
    //     (slots not in the msg are left alone -- they remain whatever the
    //     other publisher last set).
    //   - A single periodic timer at `publish_rate_hz` emits exactly one
    //     DcuCommand per cycle carrying the union of all publishers'
    //     setpoints.
    //   - Event-driven paths (on_motor_enable, on_raw, on_enable) still
    //     publish immediately because they carry one-shot transitions that
    //     must reach the STM32 within the next CAN cycle.
    //
    // Contract change for cmd publishers: the mask bit for a slot stays set
    // once anyone publishes that slot, until the channel is disabled via
    // /motor/chN/enable false. To 'release' a slot without disabling the
    // channel, publish a free-mode (kp=kd=tor=0) cmd for that slot. In
    // practice this matches what dm_hardware_interface and gripper_controller
    // already do (always publish a fixed slot subset).
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 300.0);
    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
          "publish_rate_hz=%.1f invalid, falling back to 300 Hz",
          publish_rate_hz_);
      publish_rate_hz_ = 300.0;
    }

    for (auto& c : ch_buf_) c.fill(0);
    slot_mask_.fill(0);
    cmd_received_ = false;

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    fb_sub_ = create_subscription<usb2can::msg::DcuFeedback>(
        "dcu/feedback", sensor_qos,
        std::bind(&DmMotorBridge::on_feedback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<usb2can::msg::DcuCommand>("dcu/command", 10);

    for (int ch = 0; ch < 3; ++ch) {
      if (!enable_[ch]) continue;
      const std::string ns = "motor/ch" + std::to_string(ch + 1);
      state_pub_[ch] = create_publisher<usb2can::msg::MotorStateArray>(ns + "/state", sensor_qos);
      cmd_sub_[ch]   = create_subscription<usb2can::msg::MotorCommandArray>(
          ns + "/cmd", 10,
          [this, ch](usb2can::msg::MotorCommandArray::SharedPtr msg) { on_cmd(ch, msg); });
      // 使能开关: publish std_msgs/Bool true/false
      enable_sub_[ch] = create_subscription<std_msgs::msg::Bool>(
          ns + "/enable", 10,
          [this, ch](std_msgs::msg::Bool::SharedPtr msg) { on_enable(ch, msg->data); });
      // 原始 CAN 命令 (透传 8B 任意字节, 高级用户用)
      raw_sub_[ch] = create_subscription<usb2can::msg::RawCanFrameArray>(
          ns + "/raw_cmd", 10,
          [this, ch](usb2can::msg::RawCanFrameArray::SharedPtr msg) { on_raw(ch, msg); });
      // Per-motor 使能/失能 (DM 厂家 FC/FD 命令的友好封装)
      motor_enable_sub_[ch] = create_subscription<usb2can::msg::MotorEnableArray>(
          ns + "/motor_enable", 10,
          [this, ch](usb2can::msg::MotorEnableArray::SharedPtr msg) {
            on_motor_enable(ch, msg);
          });
      RCLCPP_INFO(get_logger(),
                  "Channel %d ready (default_enable=%s): %s/{state,cmd,enable,raw_cmd,motor_enable}",
                  ch + 1, motor_enabled_[ch] ? "true" : "false", ns.c_str());
    }

    RCLCPP_INFO(get_logger(),
                "DM bridge ready. Global defaults: pos±%.1f vel±%.1f tor±%.1f "
                "Kp 0..%.0f Kd 0..%.1f",
                g_pos, g_vel, g_tor, g_kp, g_kd);

    // 列出所有"非默认"(指定了 type 或 limits 与全局不同)的电机.
    int n_custom = 0;
    for (int ch = 0; ch < 3; ++ch) {
      for (int id = 0; id < 8; ++id) {
        const auto& L = limits_[ch][id];
        const bool diff = !L.type.empty() ||
            L.pos_max != g_pos || L.vel_max != g_vel || L.tor_max != g_tor ||
            L.kp_max  != g_kp  || L.kd_max  != g_kd;
        if (!diff) continue;
        ++n_custom;
        RCLCPP_INFO(get_logger(),
            "  ch%d.id%d [%s]: pos±%.2f vel±%.1f tor±%.1f Kp 0..%.0f Kd 0..%.2f",
            ch + 1, id, L.type.empty() ? "custom" : L.type.c_str(),
            L.pos_max, L.vel_max, L.tor_max, L.kp_max, L.kd_max);
      }
    }
    if (n_custom == 0) {
      RCLCPP_INFO(get_logger(), "  (all 24 motors use global defaults)");
    }

    // Periodic aggregator: one DcuCommand per cycle, merging all publishers.
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_));
    publish_timer_ = create_wall_timer(period, [this]() {
      if (cmd_received_) {
        publish_aggregated();
      }
    });
    RCLCPP_INFO(get_logger(),
        "Aggregator timer running at %.1f Hz (override with publish_rate_hz)",
        publish_rate_hz_);
  }

private:
  // ----- 反馈解析 -----
  void on_feedback(usb2can::msg::DcuFeedback::SharedPtr msg) {
    const std::array<const std::array<uint8_t, 64>*, 3> srcs{
        reinterpret_cast<const std::array<uint8_t, 64>*>(msg->ctrl1.data()),
        reinterpret_cast<const std::array<uint8_t, 64>*>(msg->ctrl2.data()),
        reinterpret_cast<const std::array<uint8_t, 64>*>(msg->ctrl3.data()),
    };

    for (int ch = 0; ch < 3; ++ch) {
      if (!enable_[ch] || !state_pub_[ch]) continue;

      usb2can::msg::MotorStateArray out;
      out.header.stamp = msg->header.stamp;
      out.header.frame_id = msg->header.frame_id;
      out.channel = ch + 1;

      for (int slot = 0; slot < 8; ++slot) {
        const uint8_t* p = srcs[ch]->data() + slot * 8;
        unpack_motor(p, ch, slot, out.motors[slot]);
      }
      state_pub_[ch]->publish(out);
    }
  }

  void unpack_motor(const uint8_t* b, int ch, int slot, usb2can::msg::MotorState& m) {
    const auto& L = limits_[ch][slot];
    const uint32_t pos_int =  (uint32_t(b[1]) << 8) | b[2];
    const uint32_t vel_int =  (uint32_t(b[3]) << 4) | (b[4] >> 4);
    const uint32_t tor_int = ((uint32_t(b[4]) & 0x0F) << 8) | b[5];
    m.id       = b[0] & 0x0F;
    m.err      = (b[0] >> 4) & 0x0F;
    m.position = uint_to_float(pos_int, -L.pos_max, L.pos_max, 16);
    m.velocity = uint_to_float(vel_int, -L.vel_max, L.vel_max, 12);
    m.torque   = uint_to_float(tor_int, -L.tor_max, L.tor_max, 12);
    m.t_mos    = b[6];
    m.t_rotor  = b[7];
    if (m.id == 0) m.id = static_cast<uint8_t>(slot); // 字段为 0 时回退到槽位号
  }

  // ----- 命令打包 -----
  void on_cmd(int ch, usb2can::msg::MotorCommandArray::SharedPtr msg) {
    if (!enable_[ch]) return;
    if (msg->channel != 0 && msg->channel != ch + 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Received MotorCommandArray with channel=%u on /motor/ch%d/cmd topic "
          "(mismatch! routing by topic, not by msg.channel)", msg->channel, ch + 1);
    }

    // MERGE semantics: only update slots that this msg carries. Slots not
    // in the msg are left alone in the per-channel cache, so multiple
    // cooperating publishers (e.g., arm hw_interface owning slots 0..6 and
    // gripper_controller owning slot 7) can coexist on the same channel
    // without thrashing each other's setpoints.
    //
    // The cache is zero-initialized in the ctor, so any slot that has never
    // been written stays in 'free' mode (kp=kd=tor=0).
    uint8_t added_mask = 0;
    for (const auto& m : msg->motors) {
      if (m.id > 7) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ch%d motor id=%u out of range, skip", ch + 1, m.id);
        continue;
      }
      pack_motor(ch, m.id, m.position, m.velocity, m.kp, m.kd, m.torque_ff,
                 &ch_buf_[ch][m.id * 8]);
      added_mask |= static_cast<uint8_t>(1u << m.id);
    }
    // Union into existing mask. Once a slot is claimed it stays in the mask
    // (and therefore gets a CAN frame each cycle) until the channel is
    // toggled off via /motor/chN/enable. To release a slot without disabling
    // the channel, publish a free-mode cmd for it (kp=kd=tor=0) -- the bit
    // stays set but the motor receives an idle frame.
    slot_mask_[ch] |= added_mask;
    cmd_received_  = true;
    // No publish here -- the periodic aggregator timer drives the outbound
    // /dcu/command rate.
  }

  // ----- 原始 CAN 命令 (8 字节直填到指定槽位, 自动设 mask 位) -----
  // 一次性脉冲: 不污染 cmd 缓存, 不受 motor_enabled 拦截
  void on_raw(int ch, usb2can::msg::RawCanFrameArray::SharedPtr msg) {
    if (!enable_[ch]) return;

    std::array<uint8_t, 64> buf{};   // 仅本帧的 64B
    uint8_t mask = 0;
    for (const auto& f : msg->frames) {
      if (f.id > 7) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "ch%d raw id=%u out of range, skip", ch + 1, f.id);
        continue;
      }
      std::copy(f.data.begin(), f.data.end(), buf.begin() + f.id * 8);
      mask |= static_cast<uint8_t>(1u << f.id);
    }
    if (mask == 0) return;                          // 没有有效帧
    // cmd=0xFF 已重定义为 bitmask 全 1, 直接透传 mask, 不再降级.

    // 直接构造并发布一帧, 其它通道用缓存现状 (受各自 motor_enabled 控制)
    usb2can::msg::DcuCommand out;
    out.header.stamp = now();
    out.header.frame_id = "dm_motor_bridge.raw";
    auto cmd_for = [&](int c) -> uint8_t {
      return motor_enabled_[c] ? slot_mask_[c] : 0;
    };
    out.ctrl1.cmd = (ch == 0) ? mask : cmd_for(0);
    out.ctrl2.cmd = (ch == 1) ? mask : cmd_for(1);
    out.ctrl3.cmd = (ch == 2) ? mask : cmd_for(2);
    auto fill_payload = [&](auto& dst, int c) {
      const auto& src = (c == ch) ? buf : ch_buf_[c];
      std::copy(src.begin(), src.end(), dst.begin());
    };
    fill_payload(out.ctrl1.payload, 0);
    fill_payload(out.ctrl2.payload, 1);
    fill_payload(out.ctrl3.payload, 2);
    out.imu_cmd = imu_cmd_;
    cmd_pub_->publish(out);
  }

  // ----- Per-motor 使能/失能 (DM FC/FD 命令的友好封装) -----
  // 每颗电机翻译成一帧 8B CAN, 一次性合并成 1 帧 DcuCommand 发出.
  // 不污染 ch_buf_ / slot_mask_ 缓存; 与周期 /motor/chN/cmd 互不干扰.
  void on_motor_enable(int ch, usb2can::msg::MotorEnableArray::SharedPtr msg) {
    if (!enable_[ch]) return;
    if (msg->channel != 0 && msg->channel != ch + 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Received MotorEnableArray with channel=%u on /motor/ch%d/motor_enable topic "
          "(mismatch! routing by topic, not by msg.channel)", msg->channel, ch + 1);
    }

    static constexpr std::array<uint8_t, 8> DM_ENABLE  =
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static constexpr std::array<uint8_t, 8> DM_DISABLE =
        {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

    std::array<uint8_t, 64> buf{};
    uint8_t mask = 0;
    for (const auto& m : msg->motors) {
      if (m.id > 7) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "ch%d motor_enable id=%u out of range, skip", ch + 1, m.id);
        continue;
      }
      const auto& src = m.enable ? DM_ENABLE : DM_DISABLE;
      std::copy(src.begin(), src.end(), buf.begin() + m.id * 8);
      mask |= static_cast<uint8_t>(1u << m.id);
      RCLCPP_INFO(get_logger(), "ch%d motor%u -> %s",
                  ch + 1, m.id, m.enable ? "ENABLE (FC)" : "DISABLE (FD)");
    }
    if (mask == 0) return;

    // cmd=0xFF 已重定义为 bitmask 全 1 (8 颗各自收到 FC/FD 点对点), 直接透传.

    // 构造一帧 DcuCommand: 本通道用上面这个 buf+mask,
    // 其他通道沿用各自 (motor_enabled ? slot_mask : 0) 的缓存.
    usb2can::msg::DcuCommand out;
    out.header.stamp = now();
    out.header.frame_id = "dm_motor_bridge.motor_enable";
    auto cmd_for = [&](int c) -> uint8_t {
      return motor_enabled_[c] ? slot_mask_[c] : 0;
    };
    out.ctrl1.cmd = (ch == 0) ? mask : cmd_for(0);
    out.ctrl2.cmd = (ch == 1) ? mask : cmd_for(1);
    out.ctrl3.cmd = (ch == 2) ? mask : cmd_for(2);
    auto fill_payload = [&](auto& dst, int c) {
      const auto& src = (c == ch) ? buf : ch_buf_[c];
      std::copy(src.begin(), src.end(), dst.begin());
    };
    fill_payload(out.ctrl1.payload, 0);
    fill_payload(out.ctrl2.payload, 1);
    fill_payload(out.ctrl3.payload, 2);
    out.imu_cmd = imu_cmd_;
    cmd_pub_->publish(out);
  }

  void pack_motor(int ch, int slot, float pos, float vel, float kp, float kd,
                  float tff, uint8_t* b) {
    const auto& L = limits_[ch][slot];
    const uint16_t p  = float_to_uint(pos, -L.pos_max, L.pos_max, 16);
    const uint16_t v  = float_to_uint(vel, -L.vel_max, L.vel_max, 12);
    const uint16_t Kp = float_to_uint(kp,  0.0f,       L.kp_max,  12);
    const uint16_t Kd = float_to_uint(kd,  0.0f,       L.kd_max,  12);
    const uint16_t t  = float_to_uint(tff, -L.tor_max, L.tor_max, 12);
    b[0] = (p >> 8) & 0xFF;
    b[1] =  p       & 0xFF;
    b[2] = (v >> 4) & 0xFF;
    b[3] = ((v & 0x0F) << 4) | ((Kp >> 8) & 0x0F);
    b[4] =  Kp      & 0xFF;
    b[5] = (Kd >> 4) & 0xFF;
    b[6] = ((Kd & 0x0F) << 4) | ((t >> 8) & 0x0F);
    b[7] =  t       & 0xFF;
  }

  // ----- 使能切换 -----
  void on_enable(int ch, bool en) {
    if (!enable_[ch]) return;
    if (motor_enabled_[ch] == en) return;
    motor_enabled_[ch] = en;
    RCLCPP_INFO(get_logger(), "ch%d %s (Cmd=%u)", ch + 1,
                en ? "ENABLED" : "DISABLED", en ? 1u : 0u);
    // 立刻发一帧让 STM32 看到新状态
    publish_aggregated(/*force=*/true);
  }

  void publish_aggregated(bool force = false) {
    if (!force && !cmd_received_) return;
    usb2can::msg::DcuCommand out;
    out.header.stamp = now();
    out.header.frame_id = "dm_motor_bridge";

    // 每通道 cmd 字节 = (使能开关 ? slot_mask : 0)
    // slot_mask 已包含 0xFF (broadcast) / bitmask / 0 三种情况.
    auto cmd_for = [&](int ch) -> uint8_t {
      return motor_enabled_[ch] ? slot_mask_[ch] : 0;
    };
    out.ctrl1.cmd = cmd_for(0);
    out.ctrl2.cmd = cmd_for(1);
    out.ctrl3.cmd = cmd_for(2);
    std::copy(ch_buf_[0].begin(), ch_buf_[0].end(), out.ctrl1.payload.begin());
    std::copy(ch_buf_[1].begin(), ch_buf_[1].end(), out.ctrl2.payload.begin());
    std::copy(ch_buf_[2].begin(), ch_buf_[2].end(), out.ctrl3.payload.begin());
    out.imu_cmd = imu_cmd_;
    cmd_pub_->publish(out);
  }

  // params
  struct MotorLimits {
    double pos_max = 12.5;
    double vel_max = 30.0;
    double tor_max = 10.0;
    double kp_max  = 500.0;
    double kd_max  = 5.0;
    std::string type;     // 仅做日志/调试用 (例如 "DM4310")
  };
  std::array<std::array<MotorLimits, 8>, 3> limits_;   // [channel][slot]
  std::array<bool, 3> enable_;          // 通道是否启用桥接(参数)
  std::array<bool, 3> motor_enabled_;   // 当前每通道电机使能态(运行时)
  uint8_t imu_cmd_;

  // state
  std::array<std::array<uint8_t, 64>, 3> ch_buf_;   // 每通道 64B 缓存
  std::array<uint8_t, 3> slot_mask_;                // 每通道当前 cmd 字节 (mask 或 0xFF 或 0)
  bool cmd_received_;
  double publish_rate_hz_ = 300.0;                  // /dcu/command 聚合输出频率

  // ROS
  rclcpp::Subscription<usb2can::msg::DcuFeedback>::SharedPtr fb_sub_;
  rclcpp::Publisher<usb2can::msg::DcuCommand>::SharedPtr     cmd_pub_;
  rclcpp::TimerBase::SharedPtr                               publish_timer_;
  std::array<rclcpp::Publisher<usb2can::msg::MotorStateArray>::SharedPtr, 3> state_pub_;
  std::array<rclcpp::Subscription<usb2can::msg::MotorCommandArray>::SharedPtr, 3> cmd_sub_;
  std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, 3> enable_sub_;
  std::array<rclcpp::Subscription<usb2can::msg::RawCanFrameArray>::SharedPtr, 3> raw_sub_;
  std::array<rclcpp::Subscription<usb2can::msg::MotorEnableArray>::SharedPtr, 3> motor_enable_sub_;
};

}  // namespace usb2can

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usb2can::DmMotorBridge>());
  rclcpp::shutdown();
  return 0;
}
