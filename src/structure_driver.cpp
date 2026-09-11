#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "structure_core/uvc_capture.hpp"
#include "structure_core/vendor_protocol.hpp"

class StructureDriverNode : public rclcpp::Node
{
public:
  StructureDriverNode()
  : Node("structure_driver"), frame_counter_(0), hardware_mode_active_(false),
    vendor_protocol_active_(false), last_vendor_start_command_ns_(0)
  {
    frame_rate_ = declare_parameter<double>("frame_rate", 15.0);
    depth_width_ = validatePositive("depth_width", declare_parameter<int>("depth_width", 640), 640);
    depth_height_ = validatePositive("depth_height", declare_parameter<int>("depth_height", 480), 480);
    visible_width_ = validatePositive("visible_width", declare_parameter<int>("visible_width", 640), 640);
    visible_height_ = validatePositive("visible_height", declare_parameter<int>("visible_height", 480), 480);
    ir_width_ = validatePositive("ir_width", declare_parameter<int>("ir_width", 640), 640);
    ir_height_ = validatePositive("ir_height", declare_parameter<int>("ir_height", 480), 480);
    publish_if_subscribed_only_ =
      declare_parameter<bool>("publish_if_subscribed_only", true);

    io_mode_ = toLowerAscii(declare_parameter<std::string>("io_mode", "auto"));
    hardware_backend_ = toLowerAscii(declare_parameter<std::string>("hardware_backend", "auto"));
    device_name_filter_ = declare_parameter<std::string>("device_name_filter", "Structure Core");
    visible_device_index_ = declare_parameter<int>("visible_device_index", 0);
    ir_device_index_ = declare_parameter<int>("ir_device_index", -1);
    hardware_fallback_to_synthetic_ =
      declare_parameter<bool>("hardware_fallback_to_synthetic", true);
    split_stereo_frame_ = declare_parameter<bool>("split_stereo_frame", true);
    vendor_poll_timeout_ms_ = validatePositive(
      "vendor_poll_timeout_ms", declare_parameter<int>("vendor_poll_timeout_ms", 40), 40);
    vendor_no_data_reconnect_sec_ = declare_parameter<double>(
      "vendor_no_data_reconnect_sec", 6.0);
    vendor_reconnect_retry_sec_ = declare_parameter<double>(
      "vendor_reconnect_retry_sec", 1.0);
    vendor_auto_start_command_ = declare_parameter<bool>("vendor_auto_start_command", true);
    vendor_start_mode_ = toLowerAscii(
      declare_parameter<std::string>("vendor_start_mode", "frame20"));
    vendor_start_auto_cycle_ = declare_parameter<bool>("vendor_start_auto_cycle", false);
    vendor_start_auto_cycle_extended_ =
      declare_parameter<bool>("vendor_start_auto_cycle_extended", false);
    vendor_start_retry_sec_ = declare_parameter<double>("vendor_start_retry_sec", 1.5);
    vendor_start_retry_profiles_per_cycle_ =
      declare_parameter<int>("vendor_start_retry_profiles_per_cycle", 3);
    vendor_start_endpoint_ = declare_parameter<int>("vendor_start_endpoint", 1);
    vendor_start_repeat_count_ = declare_parameter<int>("vendor_start_repeat_count", 1);
    vendor_start_interval_ms_ = declare_parameter<int>("vendor_start_interval_ms", 25);
    vendor_start_preface_ = declare_parameter<bool>("vendor_start_preface", false);
    vendor_start_command_id_ = static_cast<uint32_t>(
      declare_parameter<int64_t>("vendor_start_command_id", static_cast<int64_t>(0x10000013U)));
    vendor_start_arg0_ = static_cast<uint32_t>(
      declare_parameter<int64_t>("vendor_start_arg0", static_cast<int64_t>(0x0000000fU)));
    vendor_start_arg1_ = static_cast<uint32_t>(
      declare_parameter<int64_t>("vendor_start_arg1", static_cast<int64_t>(0x00000000U)));

    depth_frame_id_ = declare_parameter<std::string>(
      "depth_frame_id", "camera_depth_optical_frame");
    visible_frame_id_ = declare_parameter<std::string>(
      "visible_frame_id", "camera_visible_optical_frame");
    left_frame_id_ = declare_parameter<std::string>(
      "left_frame_id", "camera_left_optical_frame");
    right_frame_id_ = declare_parameter<std::string>(
      "right_frame_id", "camera_right_optical_frame");

    if (frame_rate_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(), "Parameter frame_rate must be > 0.0. Falling back to 15.0 Hz.");
      frame_rate_ = 15.0;
    }

    if (io_mode_ != "auto" && io_mode_ != "hardware" && io_mode_ != "synthetic") {
      RCLCPP_WARN(
        get_logger(),
        "Parameter io_mode must be one of [auto, hardware, synthetic]. Falling back to auto.");
      io_mode_ = "auto";
    }
    if (hardware_backend_ != "auto" && hardware_backend_ != "uvc" && hardware_backend_ != "libusb") {
      RCLCPP_WARN(
        get_logger(),
        "Parameter hardware_backend must be one of [auto, uvc, libusb]. Falling back to auto.");
      hardware_backend_ = "auto";
    }
    if (
      vendor_start_mode_ != "frame20" &&
      vendor_start_mode_ != "payload12" &&
      vendor_start_mode_ != "header4" &&
      vendor_start_mode_ != "header8" &&
      vendor_start_mode_ != "frame_then_payload" &&
      vendor_start_mode_ != "broadcast_frame20" &&
      vendor_start_mode_ != "broadcast_payload12" &&
      vendor_start_mode_ != "broadcast_frame_then_payload")
    {
      RCLCPP_WARN(
        get_logger(),
        "Parameter vendor_start_mode must be one of "
        "[frame20, payload12, header4, header8, frame_then_payload, "
        "broadcast_frame20, broadcast_payload12, broadcast_frame_then_payload]. "
        "Falling back to frame20.");
      vendor_start_mode_ = "frame20";
    }
    if (vendor_start_endpoint_ <= 0 || vendor_start_endpoint_ > 0x0f) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter vendor_start_endpoint must be in [1, 15]. Falling back to 1.");
      vendor_start_endpoint_ = 1;
    }
    if (vendor_start_repeat_count_ <= 0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter vendor_start_repeat_count must be > 0. Falling back to 1.");
      vendor_start_repeat_count_ = 1;
    }
    if (vendor_start_interval_ms_ < 0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter vendor_start_interval_ms must be >= 0. Falling back to 25.");
      vendor_start_interval_ms_ = 25;
    }
    if (vendor_start_retry_profiles_per_cycle_ <= 0) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter vendor_start_retry_profiles_per_cycle must be > 0. Falling back to 1.");
      vendor_start_retry_profiles_per_cycle_ = 1;
    }

    buildVendorStartProfiles();

    depth_image_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", 10);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("depth/camera_info", 10);

    depth_color_aligned_pub_ =
      create_publisher<sensor_msgs::msg::Image>("depth_aligned/image", 10);
    depth_color_aligned_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("depth_aligned/camera_info", 10);

    depth_ir_aligned_pub_ =
      create_publisher<sensor_msgs::msg::Image>("depth_ir_aligned/image", 10);
    depth_ir_aligned_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("depth_ir_aligned/camera_info", 10);

    visible_image_pub_ = create_publisher<sensor_msgs::msg::Image>("visible/image_raw", 10);
    visible_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>("visible/camera_info", 10);

    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);

    configureIoMode();

    const auto period = std::chrono::duration<double>(1.0 / frame_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StructureDriverNode::onTimer, this));
  }

private:
  struct VendorStartProfile
  {
    std::string mode;
    int endpoint;
    bool preface;
    int repeat_count;
    int interval_ms;
    uint32_t command_id;
    uint32_t arg0;
    uint32_t arg1;
  };

  struct VendorTransferStep
  {
    uint8_t endpoint;
    std::vector<uint8_t> payload;
    int repeat_count;
  };

  static std::string toLowerAscii(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](const unsigned char c) {return static_cast<char>(std::tolower(c));});
    return value;
  }

  static void appendLe32(std::vector<uint8_t> & out, const uint32_t value)
  {
    out.push_back(static_cast<uint8_t>(value & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
  }

  VendorStartProfile makeConfiguredVendorStartProfile() const
  {
    VendorStartProfile profile;
    profile.mode = vendor_start_mode_;
    profile.endpoint = vendor_start_endpoint_;
    profile.preface = vendor_start_preface_;
    profile.repeat_count = vendor_start_repeat_count_;
    profile.interval_ms = vendor_start_interval_ms_;
    profile.command_id = vendor_start_command_id_;
    profile.arg0 = vendor_start_arg0_;
    profile.arg1 = vendor_start_arg1_;
    return profile;
  }

  static bool sameVendorStartProfile(const VendorStartProfile & a, const VendorStartProfile & b)
  {
    return
      a.mode == b.mode &&
      a.endpoint == b.endpoint &&
      a.preface == b.preface &&
      a.repeat_count == b.repeat_count &&
      a.interval_ms == b.interval_ms &&
      a.command_id == b.command_id &&
      a.arg0 == b.arg0 &&
      a.arg1 == b.arg1;
  }

  void addVendorStartProfileIfMissing(const VendorStartProfile & profile)
  {
    for (const auto & existing : vendor_start_profiles_) {
      if (sameVendorStartProfile(existing, profile)) {
        return;
      }
    }
    vendor_start_profiles_.push_back(profile);
  }

  void buildVendorStartProfiles()
  {
    vendor_start_profiles_.clear();
    vendor_start_profile_index_ = 0;

    const VendorStartProfile configured = makeConfiguredVendorStartProfile();
    addVendorStartProfileIfMissing(configured);
    if (!vendor_start_auto_cycle_) {
      return;
    }

    // Prioritize profiles that produced the strongest status responses in standalone scans.
    const auto addPriorityProfile =
      [this, &configured](
      const std::string & mode,
      const int endpoint,
      const bool preface,
      const uint32_t cmd,
      const uint32_t arg0)
      {
        VendorStartProfile p = configured;
        p.mode = mode;
        p.endpoint = endpoint;
        p.preface = preface;
        p.command_id = cmd;
        p.arg0 = arg0;
        addVendorStartProfileIfMissing(p);
      };
    addPriorityProfile("payload12", 2, true, 0x10000013U, 0x00000003U);
    addPriorityProfile("frame_then_payload", 2, true, 0x10000013U, 0x00000003U);
    addPriorityProfile("broadcast_payload12", 2, true, 0x10000013U, 0x00000003U);
    addPriorityProfile("broadcast_frame_then_payload", 2, true, 0x10000013U, 0x00000003U);
    addPriorityProfile("payload12", 2, false, 0x10000013U, 0x00000003U);
    addPriorityProfile("broadcast_payload12", 4, false, 0x10000013U, 0x0000000fU);
    addPriorityProfile("frame20", 2, false, 0x10000013U, 0x00000003U);

    const std::array<std::string, 6> sweep_modes = {
      "frame20",
      "payload12",
      "frame_then_payload",
      "broadcast_frame20",
      "broadcast_payload12",
      "broadcast_frame_then_payload"
    };
    const std::array<int, 5> sweep_eps = {1, 2, 3, 4, 5};

    for (const auto & mode : sweep_modes) {
      for (const int ep : sweep_eps) {
        VendorStartProfile p = configured;
        p.mode = mode;
        p.endpoint = ep;
        p.preface = false;
        addVendorStartProfileIfMissing(p);
      }
    }
    for (const auto & mode : sweep_modes) {
      for (const int ep : sweep_eps) {
        VendorStartProfile p = configured;
        p.mode = mode;
        p.endpoint = ep;
        p.preface = true;
        addVendorStartProfileIfMissing(p);
      }
    }
    for (uint32_t cmd = 0x10000010U; cmd <= 0x10000016U; ++cmd) {
      for (const std::string & mode : {"frame20", "payload12", "frame_then_payload"}) {
        VendorStartProfile p = configured;
        p.mode = mode;
        p.endpoint = 1;
        p.preface = false;
        p.command_id = cmd;
        addVendorStartProfileIfMissing(p);
      }
    }

    if (vendor_start_auto_cycle_extended_) {
      const std::array<uint32_t, 6> sweep_args = {0x0fU, 0x01U, 0x03U, 0x07U, 0x08U, 0x00U};
      for (uint32_t cmd = 0x10000010U; cmd <= 0x1000001aU; ++cmd) {
        for (const uint32_t arg : sweep_args) {
          for (const std::string & mode : {"frame20", "payload12", "frame_then_payload"}) {
            VendorStartProfile p = configured;
            p.command_id = cmd;
            p.arg0 = arg;
            p.endpoint = 1;
            p.preface = false;
            p.mode = mode;
            addVendorStartProfileIfMissing(p);
          }
        }
      }
    }
  }

  const VendorStartProfile & currentVendorStartProfile()
  {
    if (vendor_start_profiles_.empty()) {
      vendor_start_profiles_.push_back(makeConfiguredVendorStartProfile());
      vendor_start_profile_index_ = 0;
    }
    if (vendor_start_profile_index_ >= vendor_start_profiles_.size()) {
      vendor_start_profile_index_ = 0;
    }
    return vendor_start_profiles_[vendor_start_profile_index_];
  }

  void advanceVendorStartProfile()
  {
    if (!vendor_start_auto_cycle_ || vendor_start_profiles_.size() <= 1U) {
      return;
    }
    vendor_start_profile_index_ =
      (vendor_start_profile_index_ + 1U) % vendor_start_profiles_.size();
  }

  std::vector<uint8_t> makeVendorStartPayload(const std::string & mode, const VendorStartProfile & profile) const
  {
    std::vector<uint8_t> payload;
    if (mode == "header4") {
      payload = {0xc0U, 0xffU, 0xeeU, 0x00U};
      return payload;
    }
    if (mode == "header8") {
      payload = {0x00U, 0x00U, 0x00U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U};
      return payload;
    }
    if (mode == "payload12") {
      appendLe32(payload, profile.command_id);
      appendLe32(payload, profile.arg0);
      appendLe32(payload, profile.arg1);
      return payload;
    }

    payload = {0xc0U, 0xffU, 0xeeU, 0x00U, 0x0cU, 0x00U, 0x00U, 0x00U};
    appendLe32(payload, profile.command_id);
    appendLe32(payload, profile.arg0);
    appendLe32(payload, profile.arg1);
    return payload;
  }

  std::vector<VendorTransferStep> buildVendorStartSequence(const VendorStartProfile & profile) const
  {
    std::vector<VendorTransferStep> steps;
    const uint8_t primary_endpoint = static_cast<uint8_t>(profile.endpoint);
    const std::array<uint8_t, 5> sweep_eps = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U};

    auto appendStep = [&steps](const uint8_t endpoint, const std::vector<uint8_t> & payload, const int repeat_count) {
        if (payload.empty() || repeat_count <= 0) {
          return;
        }
        VendorTransferStep step;
        step.endpoint = endpoint;
        step.payload = payload;
        step.repeat_count = repeat_count;
        steps.push_back(step);
      };

    if (profile.preface) {
      appendStep(primary_endpoint, makeVendorStartPayload("header4", profile), 1);
      appendStep(primary_endpoint, makeVendorStartPayload("header8", profile), 1);
    }

    const std::vector<uint8_t> frame20 = makeVendorStartPayload("frame20", profile);
    const std::vector<uint8_t> payload12 = makeVendorStartPayload("payload12", profile);

    if (profile.mode == "frame20" || profile.mode == "payload12" ||
      profile.mode == "header4" || profile.mode == "header8")
    {
      appendStep(
        primary_endpoint,
        makeVendorStartPayload(profile.mode, profile),
        profile.repeat_count);
      return steps;
    }

    if (profile.mode == "frame_then_payload") {
      appendStep(primary_endpoint, frame20, profile.repeat_count);
      appendStep(primary_endpoint, payload12, profile.repeat_count);
      return steps;
    }

    if (
      profile.mode == "broadcast_frame20" ||
      profile.mode == "broadcast_payload12" ||
      profile.mode == "broadcast_frame_then_payload")
    {
      std::vector<uint8_t> ordered_eps;
      ordered_eps.reserve(sweep_eps.size());
      ordered_eps.push_back(primary_endpoint);
      for (const uint8_t ep : sweep_eps) {
        if (ep != primary_endpoint) {
          ordered_eps.push_back(ep);
        }
      }

      for (const uint8_t ep : ordered_eps) {
        if (profile.mode == "broadcast_frame20") {
          appendStep(ep, frame20, profile.repeat_count);
          continue;
        }
        if (profile.mode == "broadcast_payload12") {
          appendStep(ep, payload12, profile.repeat_count);
          continue;
        }
        appendStep(ep, frame20, profile.repeat_count);
        appendStep(ep, payload12, profile.repeat_count);
      }
      return steps;
    }

    appendStep(primary_endpoint, frame20, profile.repeat_count);
    return steps;
  }

  bool sendVendorStartCommand(const VendorStartProfile & profile)
  {
    if (!vendor_auto_start_command_ || !vendor_protocol_active_) {
      return false;
    }

    const std::vector<VendorTransferStep> steps = buildVendorStartSequence(profile);
    bool ok = !steps.empty();
    for (size_t step_index = 0; step_index < steps.size(); ++step_index) {
      const VendorTransferStep & step = steps[step_index];
      for (int i = 0; i < step.repeat_count; ++i) {
        ok = vendor_protocol_.sendRawTransfer(step.endpoint, step.payload, 200) && ok;
        const bool has_more_repeats = (i + 1) < step.repeat_count;
        const bool has_more_steps = (step_index + 1U) < steps.size();
        if ((has_more_repeats || has_more_steps) && profile.interval_ms > 0) {
          std::this_thread::sleep_for(std::chrono::milliseconds(profile.interval_ms));
        }
      }
    }
    return ok;
  }

  bool sendVendorStartCommand()
  {
    return sendVendorStartCommand(currentVendorStartProfile());
  }

  bool sendVendorStartRetryBurst()
  {
    if (!vendor_auto_start_command_ || !vendor_protocol_active_) {
      return false;
    }

    const int burst_profiles = std::max(1, vendor_start_retry_profiles_per_cycle_);
    bool any_ok = false;
    for (int i = 0; i < burst_profiles; ++i) {
      const VendorStartProfile & profile = currentVendorStartProfile();
      const bool kicked = sendVendorStartCommand(profile);
      any_ok = any_ok || kicked;

      if (!kicked) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "libusb start command resend failed. profile=%zu/%zu mode=%s ep=0x%02x cmd=0x%08x send_ep=0x%02x rc=%d",
          vendor_start_profile_index_ + 1U, vendor_start_profiles_.size(),
          profile.mode.c_str(), profile.endpoint, profile.command_id,
          vendor_protocol_.lastWriteErrorEndpoint(),
          vendor_protocol_.lastWriteErrorCode());
      } else if (vendor_start_auto_cycle_ && vendor_start_profiles_.size() > 1U) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1500,
          "libusb start auto-cycle try: profile=%zu/%zu mode=%s ep=0x%02x cmd=0x%08x",
          vendor_start_profile_index_ + 1U, vendor_start_profiles_.size(),
          profile.mode.c_str(), profile.endpoint, profile.command_id);
      }

      if (!vendor_start_auto_cycle_ || vendor_start_profiles_.size() <= 1U) {
        break;
      }
      advanceVendorStartProfile();
    }
    return any_ok;
  }

  void configureIoMode()
  {
    if (io_mode_ == "synthetic") {
      hardware_mode_active_ = false;
      RCLCPP_WARN(
        get_logger(),
        "io_mode=synthetic. Publishing software-generated streams.");
      return;
    }

    const bool try_libusb = hardware_backend_ == "auto" || hardware_backend_ == "libusb";
    const bool try_uvc = hardware_backend_ == "auto" || hardware_backend_ == "uvc";

    vendor_protocol_active_ = false;
    bool visible_open = false;
    bool ir_open = false;

    if (try_libusb) {
      vendor_protocol_active_ = vendor_protocol_.open();
      if (vendor_protocol_active_) {
        last_vendor_data_ns_ = get_clock()->now().nanoseconds();
        RCLCPP_INFO(
          get_logger(), "libusb vendor protocol connected to USB device 0x2959:0x3001.");
        if (vendor_auto_start_command_) {
          const VendorStartProfile & profile = currentVendorStartProfile();
          const bool started = sendVendorStartCommand(profile);
          last_vendor_start_command_ns_ = get_clock()->now().nanoseconds();
          RCLCPP_WARN(
            get_logger(),
            "libusb start command %s: profile=%zu/%zu mode=%s ep=0x%02x cmd=0x%08x arg0=0x%08x arg1=0x%08x send_ep=0x%02x rc=%d",
            started ? "sent" : "failed",
            vendor_start_profile_index_ + 1U,
            vendor_start_profiles_.size(),
            profile.mode.c_str(),
            profile.endpoint,
            profile.command_id, profile.arg0, profile.arg1,
            vendor_protocol_.lastWriteErrorEndpoint(),
            vendor_protocol_.lastWriteErrorCode());
          if (vendor_start_auto_cycle_ && vendor_start_profiles_.size() > 1U) {
            RCLCPP_INFO(
              get_logger(),
              "libusb start auto-cycle enabled (%zu profiles, extended=%s).",
              vendor_start_profiles_.size(),
              vendor_start_auto_cycle_extended_ ? "true" : "false");
          }
        }
      } else if (hardware_backend_ == "libusb") {
        RCLCPP_WARN(
          get_logger(),
          "hardware_backend=libusb requested, but could not open USB device 0x2959:0x3001.");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "libusb backend unavailable. Trying UVC backend.");
      }
    }

    if (!vendor_protocol_active_ && try_uvc) {
      const auto devices = structure_core::UvcCapture::listDevices();
      if (devices.empty()) {
        RCLCPP_WARN(get_logger(), "No UVC video devices detected.");
      } else {
        for (size_t i = 0; i < devices.size(); ++i) {
          RCLCPP_INFO(
            get_logger(), "Detected video device [%zu]: %s", i, devices[i].c_str());
        }
      }

      if (visible_device_index_ >= 0) {
        visible_open = visible_capture_.open(
          device_name_filter_, visible_device_index_, visible_width_, visible_height_, frame_rate_);
        if (visible_open) {
          RCLCPP_INFO(
            get_logger(), "Visible stream opened from device: %s",
            visible_capture_.deviceName().c_str());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Visible stream open failed. filter=\"%s\" index=%d",
            device_name_filter_.c_str(), visible_device_index_);
        }
      }

      if (ir_device_index_ >= 0) {
        const int requested_ir_width = split_stereo_frame_ ? (ir_width_ * 2) : ir_width_;
        ir_open = ir_capture_.open(
          device_name_filter_, ir_device_index_, requested_ir_width, ir_height_, frame_rate_);
        if (ir_open) {
          RCLCPP_INFO(
            get_logger(), "IR stream opened from device: %s",
            ir_capture_.deviceName().c_str());
        } else {
          RCLCPP_WARN(
            get_logger(),
            "IR stream open failed. filter=\"%s\" index=%d",
            device_name_filter_.c_str(), ir_device_index_);
        }
      }
    }

    hardware_mode_active_ = vendor_protocol_active_ || visible_open || ir_open;

    if (hardware_mode_active_) {
      if (vendor_protocol_active_) {
        RCLCPP_WARN(
          get_logger(),
          "libusb hardware mode active. Raw endpoint bytes are converted to pseudo image/depth streams.");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "UVC hardware mode active. Depth streams are pseudo-depth derived from image intensity.");
      }
      return;
    }

    if (io_mode_ == "hardware" && !hardware_fallback_to_synthetic_) {
      throw std::runtime_error(
              "io_mode=hardware but no hardware stream opened and hardware_fallback_to_synthetic=false");
    }

    if (io_mode_ == "hardware") {
      RCLCPP_WARN(
        get_logger(),
        "io_mode=hardware requested, but no stream could be opened. Falling back to synthetic mode.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "io_mode=auto could not open hardware stream. Falling back to synthetic mode.");
    }
  }

  bool reconnectVendorProtocol()
  {
    vendor_protocol_.close();
    vendor_protocol_active_ = vendor_protocol_.open();
    if (!vendor_protocol_active_) {
      return false;
    }
    if (vendor_auto_start_command_) {
      const VendorStartProfile & profile = currentVendorStartProfile();
      (void)sendVendorStartCommand(profile);
      last_vendor_start_command_ns_ = get_clock()->now().nanoseconds();
      RCLCPP_INFO(
        get_logger(),
        "libusb reconnect start command: profile=%zu/%zu mode=%s ep=0x%02x cmd=0x%08x arg0=0x%08x arg1=0x%08x send_ep=0x%02x rc=%d",
        vendor_start_profile_index_ + 1U, vendor_start_profiles_.size(),
        profile.mode.c_str(), profile.endpoint, profile.command_id, profile.arg0, profile.arg1,
        vendor_protocol_.lastWriteErrorEndpoint(), vendor_protocol_.lastWriteErrorCode());
    }
    last_vendor_data_ns_ = get_clock()->now().nanoseconds();
    return true;
  }

  bool tryRecoverVendorProtocol()
  {
    if (vendor_protocol_active_) {
      return true;
    }
    if (io_mode_ == "synthetic") {
      return false;
    }
    if (hardware_backend_ != "auto" && hardware_backend_ != "libusb") {
      return false;
    }

    const int64_t now_ns = get_clock()->now().nanoseconds();
    if (
      vendor_reconnect_retry_sec_ > 0.0 &&
      (now_ns - last_vendor_reconnect_attempt_ns_) <
      static_cast<int64_t>(vendor_reconnect_retry_sec_ * 1000000000.0))
    {
      return false;
    }
    last_vendor_reconnect_attempt_ns_ = now_ns;

    if (!reconnectVendorProtocol()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "libusb backend reopen retry failed.");
      return false;
    }

    hardware_mode_active_ = true;
    RCLCPP_INFO(
      get_logger(),
      "libusb backend reopen retry succeeded.");
    return true;
  }

  bool hasSubscribers(const rclcpp::PublisherBase::SharedPtr & publisher) const
  {
    return
      (publisher->get_subscription_count() +
      publisher->get_intra_process_subscription_count()) > 0U;
  }

  builtin_interfaces::msg::Time toBuiltinTime(const rclcpp::Time & time) const
  {
    const int64_t nanoseconds = time.nanoseconds();
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return msg;
  }

  int validatePositive(const char * name, const int value, const int fallback) const
  {
    if (value > 0) {
      return value;
    }
    RCLCPP_WARN(
      get_logger(), "Parameter %s must be > 0. Falling back to %d.", name, fallback);
    return fallback;
  }

  sensor_msgs::msg::CameraInfo makeCameraInfo(
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp,
    const int width,
    const int height) const
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.header.stamp = stamp;
    info.height = static_cast<uint32_t>(height);
    info.width = static_cast<uint32_t>(width);
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    const double fx = static_cast<double>(width) * 0.9;
    const double fy = static_cast<double>(height) * 0.9;
    const double cx = (static_cast<double>(width) - 1.0) * 0.5;
    const double cy = (static_cast<double>(height) - 1.0) * 0.5;

    info.k = {
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    };
    info.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };
    info.p = {
      fx, 0.0, cx, 0.0,
      0.0, fy, cy, 0.0,
      0.0, 0.0, 1.0, 0.0
    };
    return info;
  }

  std::vector<uint16_t> generateDepthFrame(const int width, const int height) const
  {
    std::vector<uint16_t> depth(static_cast<size_t>(width) * static_cast<size_t>(height), 0U);
    const double t = static_cast<double>(frame_counter_) * 0.05;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const double a = std::sin((static_cast<double>(x) * 0.02) + t);
        const double b = std::cos((static_cast<double>(y) * 0.03) + 0.7 * t);
        const double mm = 1200.0 + 350.0 * a + 250.0 * b;
        const double clamped = std::max(300.0, std::min(4000.0, mm));
        depth[static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)] =
          static_cast<uint16_t>(clamped);
      }
    }
    return depth;
  }

  std::vector<uint16_t> resizeDepthNearest(
    const std::vector<uint16_t> & src,
    const int src_w,
    const int src_h,
    const int dst_w,
    const int dst_h) const
  {
    std::vector<uint16_t> dst(static_cast<size_t>(dst_w) * static_cast<size_t>(dst_h), 0U);
    for (int y = 0; y < dst_h; ++y) {
      const int sy = std::min(src_h - 1, (y * src_h) / dst_h);
      for (int x = 0; x < dst_w; ++x) {
        const int sx = std::min(src_w - 1, (x * src_w) / dst_w);
        dst[static_cast<size_t>(y) * static_cast<size_t>(dst_w) + static_cast<size_t>(x)] =
          src[static_cast<size_t>(sy) * static_cast<size_t>(src_w) + static_cast<size_t>(sx)];
      }
    }
    return dst;
  }

  std::vector<uint8_t> resizeMonoNearest(
    const std::vector<uint8_t> & src,
    const int src_w,
    const int src_h,
    const int dst_w,
    const int dst_h) const
  {
    std::vector<uint8_t> dst(static_cast<size_t>(dst_w) * static_cast<size_t>(dst_h), 0U);
    for (int y = 0; y < dst_h; ++y) {
      const int sy = std::min(src_h - 1, (y * src_h) / dst_h);
      for (int x = 0; x < dst_w; ++x) {
        const int sx = std::min(src_w - 1, (x * src_w) / dst_w);
        dst[static_cast<size_t>(y) * static_cast<size_t>(dst_w) + static_cast<size_t>(x)] =
          src[static_cast<size_t>(sy) * static_cast<size_t>(src_w) + static_cast<size_t>(sx)];
      }
    }
    return dst;
  }

  std::vector<uint8_t> generateVisibleRGB(const int width, const int height) const
  {
    std::vector<uint8_t> rgb(
      static_cast<size_t>(width) * static_cast<size_t>(height) * static_cast<size_t>(3), 0U);
    const int phase = frame_counter_ % 255;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t idx =
          (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 3U;
        rgb[idx + 0U] = static_cast<uint8_t>((x + phase) % 255);
        rgb[idx + 1U] = static_cast<uint8_t>((y + phase) % 255);
        rgb[idx + 2U] = static_cast<uint8_t>((x + y + phase) % 255);
      }
    }
    return rgb;
  }

  std::vector<uint8_t> depthToMono8(
    const std::vector<uint16_t> & depth_mm,
    const int offset) const
  {
    std::vector<uint8_t> out(depth_mm.size(), 0U);
    for (size_t i = 0; i < depth_mm.size(); ++i) {
      int value = static_cast<int>((depth_mm[i] - 300U) / 14U) + offset;
      value = std::max(0, std::min(255, value));
      out[i] = static_cast<uint8_t>(value);
    }
    return out;
  }

  std::vector<uint8_t> rgbToMono8(const std::vector<uint8_t> & rgb) const
  {
    std::vector<uint8_t> mono(rgb.size() / 3U, 0U);
    for (size_t i = 0, j = 0; i < rgb.size(); i += 3U, ++j) {
      const int r = rgb[i + 0U];
      const int g = rgb[i + 1U];
      const int b = rgb[i + 2U];
      mono[j] = static_cast<uint8_t>((77 * r + 150 * g + 29 * b) >> 8);
    }
    return mono;
  }

  std::vector<uint8_t> monoToRGB(const std::vector<uint8_t> & mono) const
  {
    std::vector<uint8_t> rgb(mono.size() * 3U, 0U);
    for (size_t i = 0; i < mono.size(); ++i) {
      const uint8_t value = mono[i];
      rgb[i * 3U + 0U] = value;
      rgb[i * 3U + 1U] = value;
      rgb[i * 3U + 2U] = value;
    }
    return rgb;
  }

  std::vector<uint16_t> monoToPseudoDepth(const std::vector<uint8_t> & mono) const
  {
    std::vector<uint16_t> depth(mono.size(), 0U);
    for (size_t i = 0; i < mono.size(); ++i) {
      const int value = static_cast<int>(mono[i]);
      const int mm = 300 + ((255 - value) * 3700) / 255;
      depth[i] = static_cast<uint16_t>(mm);
    }
    return depth;
  }

  bool frameToRgbAndMono(
    const structure_core::UvcFrame & frame,
    std::vector<uint8_t> & rgb_out,
    std::vector<uint8_t> & mono_out) const
  {
    if (!frame.valid()) {
      return false;
    }

    const size_t pixel_count = static_cast<size_t>(frame.width) * static_cast<size_t>(frame.height);
    if (frame.encoding == "rgb8") {
      if (frame.data.size() != (pixel_count * 3U)) {
        return false;
      }
      rgb_out = frame.data;
      mono_out = rgbToMono8(frame.data);
      return true;
    }

    if (frame.encoding == "mono8") {
      if (frame.data.size() != pixel_count) {
        return false;
      }
      mono_out = frame.data;
      rgb_out = monoToRGB(frame.data);
      return true;
    }

    return false;
  }

  void splitStereoFromMono(
    const std::vector<uint8_t> & mono,
    const int width,
    const int height,
    std::vector<uint8_t> & left_out,
    std::vector<uint8_t> & right_out,
    int & out_width,
    int & out_height) const
  {
    if (mono.empty() || width <= 1 || height <= 0) {
      left_out.clear();
      right_out.clear();
      out_width = 0;
      out_height = 0;
      return;
    }

    if (split_stereo_frame_ && width >= 2) {
      const int single_width = width / 2;
      left_out.assign(static_cast<size_t>(single_width) * static_cast<size_t>(height), 0U);
      right_out.assign(static_cast<size_t>(single_width) * static_cast<size_t>(height), 0U);

      for (int y = 0; y < height; ++y) {
        const size_t row = static_cast<size_t>(y) * static_cast<size_t>(width);
        const size_t out_row = static_cast<size_t>(y) * static_cast<size_t>(single_width);

        // Right stream is first half, left stream is second half.
        std::copy(
          mono.begin() + static_cast<std::ptrdiff_t>(row),
          mono.begin() + static_cast<std::ptrdiff_t>(row + static_cast<size_t>(single_width)),
          right_out.begin() + static_cast<std::ptrdiff_t>(out_row));

        std::copy(
          mono.begin() + static_cast<std::ptrdiff_t>(row + static_cast<size_t>(single_width)),
          mono.begin() + static_cast<std::ptrdiff_t>(row + static_cast<size_t>(single_width * 2)),
          left_out.begin() + static_cast<std::ptrdiff_t>(out_row));
      }

      out_width = single_width;
      out_height = height;
      return;
    }

    left_out = mono;
    right_out = mono;
    out_width = width;
    out_height = height;
  }

  const std::vector<uint8_t> * pickFirstNonEmpty(
    const std::vector<uint8_t> & a,
    const std::vector<uint8_t> & b,
    const std::vector<uint8_t> & c,
    const std::vector<uint8_t> & d,
    const std::vector<uint8_t> & e) const
  {
    if (!a.empty()) {
      return &a;
    }
    if (!b.empty()) {
      return &b;
    }
    if (!c.empty()) {
      return &c;
    }
    if (!d.empty()) {
      return &d;
    }
    if (!e.empty()) {
      return &e;
    }
    return nullptr;
  }

  std::vector<uint8_t> bytesToMonoFrame(
    const std::vector<uint8_t> & bytes,
    const int width,
    const int height) const
  {
    if (bytes.empty() || width <= 0 || height <= 0) {
      return {};
    }

    const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
    std::vector<uint8_t> mono(pixel_count, 0U);

    size_t offset = 0U;
    if (
      bytes.size() >= 8U &&
      bytes[0] == 0xc0U && bytes[1] == 0xffU && bytes[2] == 0xeeU && bytes[3] == 0x00U)
    {
      // Skip the command-style framing header if it is present.
      offset = 8U;
    }
    const size_t payload_offset = std::min(offset, bytes.size() - 1U);

    if (bytes.size() >= (payload_offset + pixel_count * 2U)) {
      for (size_t i = 0; i < pixel_count; ++i) {
        mono[i] = bytes[payload_offset + i * 2U];
      }
      return mono;
    }

    const size_t usable = std::max<size_t>(1U, bytes.size() - payload_offset);
    for (size_t i = 0; i < pixel_count; ++i) {
      mono[i] = bytes[payload_offset + (i % usable)];
    }
    return mono;
  }

  sensor_msgs::msg::Image makeDepthImage(
    const std::vector<uint16_t> & depth_mm,
    const int width,
    const int height,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.encoding = "16UC1";
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(
      width * static_cast<int>(sizeof(uint16_t)));
    msg.is_bigendian = 0;
    msg.data.resize(static_cast<size_t>(height) * static_cast<size_t>(msg.step), 0U);
    std::memcpy(msg.data.data(), depth_mm.data(), msg.data.size());
    return msg;
  }

  sensor_msgs::msg::Image makeMono8Image(
    const std::vector<uint8_t> & mono,
    const int width,
    const int height,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.encoding = "mono8";
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width);
    msg.is_bigendian = 0;
    msg.data = mono;
    return msg;
  }

  sensor_msgs::msg::Image makeRGBImage(
    const std::vector<uint8_t> & rgb,
    const int width,
    const int height,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.encoding = "rgb8";
    msg.height = static_cast<uint32_t>(height);
    msg.width = static_cast<uint32_t>(width);
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(width * 3);
    msg.is_bigendian = 0;
    msg.data = rgb;
    return msg;
  }

  bool publishVendorProtocol(const builtin_interfaces::msg::Time & stamp)
  {
    if (!vendor_protocol_active_) {
      return false;
    }

    structure_core::VendorPollResult polled;
    if (!vendor_protocol_.poll(polled, vendor_poll_timeout_ms_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "libusb poll failed. ep=0x%02x rc=%d. Hardware USB backend may have disconnected.",
        vendor_protocol_.lastReadErrorEndpoint(),
        vendor_protocol_.lastReadErrorCode());
      if (vendor_start_auto_cycle_) {
        advanceVendorStartProfile();
      }
      if (reconnectVendorProtocol()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "libusb backend recovered after poll failure.");
      }
      return false;
    }

    if (
      polled.endpoint81.empty() && polled.endpoint82.empty() && polled.endpoint83.empty() &&
      polled.endpoint84.empty() && polled.endpoint85.empty())
    {
      const int64_t now_ns = get_clock()->now().nanoseconds();
      if (vendor_auto_start_command_) {
        const int64_t retry_ns = std::max<int64_t>(
          1LL, static_cast<int64_t>(vendor_start_retry_sec_ * 1000000000.0));
        if ((now_ns - last_vendor_start_command_ns_) > retry_ns) {
          const bool kicked = sendVendorStartRetryBurst();
          last_vendor_start_command_ns_ = now_ns;
          if (!kicked && !vendor_start_auto_cycle_) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "libusb start command resend failed. send_ep=0x%02x rc=%d",
              vendor_protocol_.lastWriteErrorEndpoint(),
              vendor_protocol_.lastWriteErrorCode());
          }
        }
      }
      if (
        vendor_no_data_reconnect_sec_ > 0.0 &&
        (now_ns - last_vendor_data_ns_) >
        static_cast<int64_t>(vendor_no_data_reconnect_sec_ * 1000000000.0))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "No USB payload for %.1f sec. Reopening libusb backend.",
          vendor_no_data_reconnect_sec_);
        if (reconnectVendorProtocol()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "libusb backend reconnected.");
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "libusb backend reopen failed.");
        }
      }
      return false;
    }

    if (!polled.endpoint83.empty() || !polled.endpoint84.empty() || !polled.endpoint85.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "libusb stream payload detected: ep83=%zu ep84=%zu ep85=%zu (profile=%zu/%zu)",
        polled.endpoint83.size(), polled.endpoint84.size(), polled.endpoint85.size(),
        vendor_start_profile_index_ + 1U, vendor_start_profiles_.size());
    } else if (!polled.endpoint81.empty() || !polled.endpoint82.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "libusb status payload detected: ep81=%zu ep82=%zu (profile=%zu/%zu)",
        polled.endpoint81.size(), polled.endpoint82.size(),
        vendor_start_profile_index_ + 1U, vendor_start_profiles_.size());
    }

    last_vendor_data_ns_ = get_clock()->now().nanoseconds();

    const std::vector<uint8_t> * visible_payload = pickFirstNonEmpty(
      polled.endpoint83, polled.endpoint84, polled.endpoint85, polled.endpoint82, polled.endpoint81);
    std::vector<uint8_t> visible_mono;
    std::vector<uint8_t> visible_rgb;
    int visible_w = 0;
    int visible_h = 0;
    if (visible_payload != nullptr) {
      visible_w = visible_width_;
      visible_h = visible_height_;
      visible_mono = bytesToMonoFrame(*visible_payload, visible_w, visible_h);
      visible_rgb = monoToRGB(visible_mono);
    }

    const std::vector<uint8_t> * left_payload = pickFirstNonEmpty(
      polled.endpoint84, polled.endpoint83, polled.endpoint85, polled.endpoint82, polled.endpoint81);
    const std::vector<uint8_t> * right_payload = pickFirstNonEmpty(
      polled.endpoint85, polled.endpoint83, polled.endpoint84, polled.endpoint82, polled.endpoint81);

    std::vector<uint8_t> left_mono;
    std::vector<uint8_t> right_mono;
    int ir_w = 0;
    int ir_h = 0;
    if (left_payload != nullptr || right_payload != nullptr) {
      ir_w = ir_width_;
      ir_h = ir_height_;
      if (left_payload != nullptr) {
        left_mono = bytesToMonoFrame(*left_payload, ir_w, ir_h);
      }
      if (right_payload != nullptr) {
        right_mono = bytesToMonoFrame(*right_payload, ir_w, ir_h);
      }
      if (left_mono.empty() && !right_mono.empty()) {
        left_mono = right_mono;
      }
      if (right_mono.empty() && !left_mono.empty()) {
        right_mono = left_mono;
      }
    }

    if (left_mono.empty() && !visible_mono.empty()) {
      left_mono = resizeMonoNearest(visible_mono, visible_w, visible_h, ir_width_, ir_height_);
      right_mono = left_mono;
      ir_w = ir_width_;
      ir_h = ir_height_;
    }

    std::vector<uint16_t> depth_native;
    if (!right_mono.empty()) {
      auto depth = monoToPseudoDepth(right_mono);
      if (ir_w != depth_width_ || ir_h != depth_height_) {
        depth_native = resizeDepthNearest(depth, ir_w, ir_h, depth_width_, depth_height_);
      } else {
        depth_native = std::move(depth);
      }
    } else if (!visible_mono.empty()) {
      auto depth = monoToPseudoDepth(visible_mono);
      if (visible_w != depth_width_ || visible_h != depth_height_) {
        depth_native = resizeDepthNearest(depth, visible_w, visible_h, depth_width_, depth_height_);
      } else {
        depth_native = std::move(depth);
      }
    }

    const bool want_depth =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_image_pub_) || hasSubscribers(depth_info_pub_);
    if (want_depth && !depth_native.empty()) {
      depth_image_pub_->publish(
        makeDepthImage(depth_native, depth_width_, depth_height_, depth_frame_id_, stamp));
      depth_info_pub_->publish(makeCameraInfo(depth_frame_id_, stamp, depth_width_, depth_height_));
    }

    const bool want_visible =
      !publish_if_subscribed_only_ ||
      hasSubscribers(visible_image_pub_) || hasSubscribers(visible_info_pub_);
    if (want_visible && !visible_rgb.empty() && visible_w > 0 && visible_h > 0) {
      visible_image_pub_->publish(
        makeRGBImage(visible_rgb, visible_w, visible_h, visible_frame_id_, stamp));
      visible_info_pub_->publish(makeCameraInfo(visible_frame_id_, stamp, visible_w, visible_h));
    }

    const bool want_depth_aligned =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_color_aligned_pub_) || hasSubscribers(depth_color_aligned_info_pub_);
    if (want_depth_aligned && !depth_native.empty() && visible_w > 0 && visible_h > 0) {
      const auto aligned = resizeDepthNearest(
        depth_native, depth_width_, depth_height_, visible_w, visible_h);
      depth_color_aligned_pub_->publish(
        makeDepthImage(aligned, visible_w, visible_h, visible_frame_id_, stamp));
      depth_color_aligned_info_pub_->publish(
        makeCameraInfo(visible_frame_id_, stamp, visible_w, visible_h));
    }

    const bool want_ir =
      !publish_if_subscribed_only_ ||
      hasSubscribers(left_image_pub_) || hasSubscribers(left_info_pub_) ||
      hasSubscribers(right_image_pub_) || hasSubscribers(right_info_pub_);
    if (want_ir && !left_mono.empty() && !right_mono.empty() && ir_w > 0 && ir_h > 0) {
      left_image_pub_->publish(
        makeMono8Image(left_mono, ir_w, ir_h, left_frame_id_, stamp));
      left_info_pub_->publish(makeCameraInfo(left_frame_id_, stamp, ir_w, ir_h));

      right_image_pub_->publish(
        makeMono8Image(right_mono, ir_w, ir_h, right_frame_id_, stamp));
      right_info_pub_->publish(makeCameraInfo(right_frame_id_, stamp, ir_w, ir_h));
    }

    const bool want_depth_ir_aligned =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_ir_aligned_pub_) || hasSubscribers(depth_ir_aligned_info_pub_);
    if (want_depth_ir_aligned && !depth_native.empty() && ir_w > 0 && ir_h > 0) {
      const auto ir_aligned = resizeDepthNearest(
        depth_native, depth_width_, depth_height_, ir_w, ir_h);
      depth_ir_aligned_pub_->publish(
        makeDepthImage(ir_aligned, ir_w, ir_h, depth_frame_id_, stamp));
      depth_ir_aligned_info_pub_->publish(
        makeCameraInfo(depth_frame_id_, stamp, ir_w, ir_h));
    }

    return true;
  }

  bool publishHardware(const builtin_interfaces::msg::Time & stamp)
  {
    structure_core::UvcFrame visible_frame;
    const bool got_visible_frame =
      visible_capture_.isOpen() && visible_capture_.getLatestFrame(visible_frame);

    structure_core::UvcFrame ir_frame;
    const bool got_ir_frame = ir_capture_.isOpen() && ir_capture_.getLatestFrame(ir_frame);

    if (!got_visible_frame && !got_ir_frame) {
      return false;
    }

    std::vector<uint8_t> visible_rgb;
    std::vector<uint8_t> visible_mono;
    int visible_w = 0;
    int visible_h = 0;

    if (got_visible_frame) {
      if (!frameToRgbAndMono(visible_frame, visible_rgb, visible_mono)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping visible frame: unsupported format or unexpected size.");
      } else {
        visible_w = visible_frame.width;
        visible_h = visible_frame.height;
      }
    }

    std::vector<uint8_t> left_mono;
    std::vector<uint8_t> right_mono;
    int ir_w = 0;
    int ir_h = 0;

    if (got_ir_frame) {
      std::vector<uint8_t> ir_rgb_unused;
      std::vector<uint8_t> ir_mono;
      if (!frameToRgbAndMono(ir_frame, ir_rgb_unused, ir_mono)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Dropping IR frame: unsupported format or unexpected size.");
      } else {
        splitStereoFromMono(ir_mono, ir_frame.width, ir_frame.height, left_mono, right_mono, ir_w, ir_h);
      }
    }

    if (left_mono.empty() && !visible_mono.empty()) {
      left_mono = resizeMonoNearest(visible_mono, visible_w, visible_h, ir_width_, ir_height_);
      right_mono = left_mono;
      ir_w = ir_width_;
      ir_h = ir_height_;
    }

    std::vector<uint8_t> depth_seed_mono;
    int depth_seed_w = 0;
    int depth_seed_h = 0;
    if (!right_mono.empty()) {
      depth_seed_mono = right_mono;
      depth_seed_w = ir_w;
      depth_seed_h = ir_h;
    } else if (!visible_mono.empty()) {
      depth_seed_mono = visible_mono;
      depth_seed_w = visible_w;
      depth_seed_h = visible_h;
    }

    std::vector<uint16_t> depth_native;
    if (!depth_seed_mono.empty()) {
      auto depth = monoToPseudoDepth(depth_seed_mono);
      if (depth_seed_w != depth_width_ || depth_seed_h != depth_height_) {
        depth_native = resizeDepthNearest(depth, depth_seed_w, depth_seed_h, depth_width_, depth_height_);
      } else {
        depth_native = std::move(depth);
      }
    }

    const bool want_depth =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_image_pub_) || hasSubscribers(depth_info_pub_);
    if (want_depth && !depth_native.empty()) {
      depth_image_pub_->publish(
        makeDepthImage(depth_native, depth_width_, depth_height_, depth_frame_id_, stamp));
      depth_info_pub_->publish(makeCameraInfo(depth_frame_id_, stamp, depth_width_, depth_height_));
    }

    const bool want_visible =
      !publish_if_subscribed_only_ ||
      hasSubscribers(visible_image_pub_) || hasSubscribers(visible_info_pub_);
    if (want_visible && !visible_rgb.empty() && visible_w > 0 && visible_h > 0) {
      visible_image_pub_->publish(
        makeRGBImage(visible_rgb, visible_w, visible_h, visible_frame_id_, stamp));
      visible_info_pub_->publish(makeCameraInfo(visible_frame_id_, stamp, visible_w, visible_h));
    }

    const bool want_depth_aligned =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_color_aligned_pub_) || hasSubscribers(depth_color_aligned_info_pub_);
    if (want_depth_aligned && !depth_native.empty() && visible_w > 0 && visible_h > 0) {
      const auto aligned = resizeDepthNearest(
        depth_native, depth_width_, depth_height_, visible_w, visible_h);
      depth_color_aligned_pub_->publish(
        makeDepthImage(aligned, visible_w, visible_h, visible_frame_id_, stamp));
      depth_color_aligned_info_pub_->publish(
        makeCameraInfo(visible_frame_id_, stamp, visible_w, visible_h));
    }

    const bool want_ir =
      !publish_if_subscribed_only_ ||
      hasSubscribers(left_image_pub_) || hasSubscribers(left_info_pub_) ||
      hasSubscribers(right_image_pub_) || hasSubscribers(right_info_pub_);
    if (want_ir && !left_mono.empty() && !right_mono.empty() && ir_w > 0 && ir_h > 0) {
      left_image_pub_->publish(
        makeMono8Image(left_mono, ir_w, ir_h, left_frame_id_, stamp));
      left_info_pub_->publish(makeCameraInfo(left_frame_id_, stamp, ir_w, ir_h));

      right_image_pub_->publish(
        makeMono8Image(right_mono, ir_w, ir_h, right_frame_id_, stamp));
      right_info_pub_->publish(makeCameraInfo(right_frame_id_, stamp, ir_w, ir_h));
    }

    const bool want_depth_ir_aligned =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_ir_aligned_pub_) || hasSubscribers(depth_ir_aligned_info_pub_);
    if (want_depth_ir_aligned && !depth_native.empty() && ir_w > 0 && ir_h > 0) {
      const auto ir_aligned = resizeDepthNearest(
        depth_native, depth_width_, depth_height_, ir_w, ir_h);
      depth_ir_aligned_pub_->publish(
        makeDepthImage(ir_aligned, ir_w, ir_h, depth_frame_id_, stamp));
      depth_ir_aligned_info_pub_->publish(
        makeCameraInfo(depth_frame_id_, stamp, ir_w, ir_h));
    }

    return true;
  }

  void publishSynthetic(const builtin_interfaces::msg::Time & stamp)
  {
    const std::vector<uint16_t> depth_native = generateDepthFrame(depth_width_, depth_height_);

    const bool want_depth =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_image_pub_) || hasSubscribers(depth_info_pub_);
    if (want_depth) {
      const auto depth_msg = makeDepthImage(
        depth_native, depth_width_, depth_height_, depth_frame_id_, stamp);
      depth_image_pub_->publish(depth_msg);
      depth_info_pub_->publish(makeCameraInfo(depth_frame_id_, stamp, depth_width_, depth_height_));
    }

    const bool want_visible =
      !publish_if_subscribed_only_ ||
      hasSubscribers(visible_image_pub_) || hasSubscribers(visible_info_pub_);
    if (want_visible) {
      const auto rgb = generateVisibleRGB(visible_width_, visible_height_);
      visible_image_pub_->publish(
        makeRGBImage(rgb, visible_width_, visible_height_, visible_frame_id_, stamp));
      visible_info_pub_->publish(
        makeCameraInfo(visible_frame_id_, stamp, visible_width_, visible_height_));
    }

    const bool want_depth_aligned =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_color_aligned_pub_) || hasSubscribers(depth_color_aligned_info_pub_);
    if (want_depth_aligned) {
      const auto aligned = resizeDepthNearest(
        depth_native, depth_width_, depth_height_, visible_width_, visible_height_);
      depth_color_aligned_pub_->publish(
        makeDepthImage(aligned, visible_width_, visible_height_, visible_frame_id_, stamp));
      depth_color_aligned_info_pub_->publish(
        makeCameraInfo(visible_frame_id_, stamp, visible_width_, visible_height_));
    }

    const auto ir_depth = resizeDepthNearest(
      depth_native, depth_width_, depth_height_, ir_width_, ir_height_);

    const bool want_ir =
      !publish_if_subscribed_only_ ||
      hasSubscribers(left_image_pub_) || hasSubscribers(left_info_pub_) ||
      hasSubscribers(right_image_pub_) || hasSubscribers(right_info_pub_);
    if (want_ir) {
      left_image_pub_->publish(
        makeMono8Image(depthToMono8(ir_depth, 10), ir_width_, ir_height_, left_frame_id_, stamp));
      left_info_pub_->publish(makeCameraInfo(left_frame_id_, stamp, ir_width_, ir_height_));

      right_image_pub_->publish(
        makeMono8Image(depthToMono8(ir_depth, -10), ir_width_, ir_height_, right_frame_id_, stamp));
      right_info_pub_->publish(makeCameraInfo(right_frame_id_, stamp, ir_width_, ir_height_));
    }

    const bool want_depth_ir_aligned =
      !publish_if_subscribed_only_ ||
      hasSubscribers(depth_ir_aligned_pub_) || hasSubscribers(depth_ir_aligned_info_pub_);
    if (want_depth_ir_aligned) {
      depth_ir_aligned_pub_->publish(
        makeDepthImage(ir_depth, ir_width_, ir_height_, depth_frame_id_, stamp));
      depth_ir_aligned_info_pub_->publish(
        makeCameraInfo(depth_frame_id_, stamp, ir_width_, ir_height_));
    }
  }

  void onTimer()
  {
    const builtin_interfaces::msg::Time stamp = toBuiltinTime(get_clock()->now());

    (void)tryRecoverVendorProtocol();

    if (hardware_mode_active_) {
      if (vendor_protocol_active_ && publishVendorProtocol(stamp)) {
        ++frame_counter_;
        return;
      }

      if (publishHardware(stamp)) {
        ++frame_counter_;
        return;
      }

      if (!hardware_fallback_to_synthetic_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Hardware mode active but no frame received yet. fallback disabled.");
        ++frame_counter_;
        return;
      }

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Hardware mode active but no frame received yet. Using synthetic fallback.");
    }

    publishSynthetic(stamp);
    ++frame_counter_;
  }

  double frame_rate_;
  int depth_width_;
  int depth_height_;
  int visible_width_;
  int visible_height_;
  int ir_width_;
  int ir_height_;
  bool publish_if_subscribed_only_;
  std::string io_mode_;
  std::string hardware_backend_;
  std::string device_name_filter_;
  int visible_device_index_;
  int ir_device_index_;
  bool hardware_fallback_to_synthetic_;
  bool split_stereo_frame_;
  int vendor_poll_timeout_ms_;
  double vendor_no_data_reconnect_sec_;
  double vendor_reconnect_retry_sec_;
  bool vendor_auto_start_command_;
  std::string vendor_start_mode_;
  bool vendor_start_auto_cycle_;
  bool vendor_start_auto_cycle_extended_;
  double vendor_start_retry_sec_;
  int vendor_start_retry_profiles_per_cycle_;
  int vendor_start_endpoint_;
  int vendor_start_repeat_count_;
  int vendor_start_interval_ms_;
  bool vendor_start_preface_;
  std::vector<VendorStartProfile> vendor_start_profiles_;
  size_t vendor_start_profile_index_{0U};
  uint32_t vendor_start_command_id_;
  uint32_t vendor_start_arg0_;
  uint32_t vendor_start_arg1_;
  std::string depth_frame_id_;
  std::string visible_frame_id_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  int frame_counter_;
  bool hardware_mode_active_;
  bool vendor_protocol_active_;
  int64_t last_vendor_start_command_ns_;
  int64_t last_vendor_data_ns_{0};
  int64_t last_vendor_reconnect_attempt_ns_{0};
  structure_core::VendorProtocol vendor_protocol_;
  structure_core::UvcCapture visible_capture_;
  structure_core::UvcCapture ir_capture_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_color_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_color_aligned_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_ir_aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_ir_aligned_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visible_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr visible_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StructureDriverNode>());
  rclcpp::shutdown();
  return 0;
}
