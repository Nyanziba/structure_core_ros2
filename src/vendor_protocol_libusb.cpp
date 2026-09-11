#include "structure_core/vendor_protocol.hpp"

#include <algorithm>
#include <cstddef>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#if __has_include(<libusb.h>)
#include <libusb.h>
#elif __has_include(<libusb-1.0/libusb.h>)
#include <libusb-1.0/libusb.h>
#else
#error "libusb headers not found"
#endif

namespace structure_core
{

namespace
{

constexpr uint16_t kVendorId = 0x2959;
constexpr uint16_t kProductId = 0x3001;
constexpr uint8_t kCommandOutEndpoint = 0x01;
constexpr std::array<uint8_t, 5> kInputEndpoints = {0x81, 0x82, 0x83, 0x84, 0x85};

enum class EndpointReadState
{
  Ok = 0,
  TransientError = 1,
  FatalError = 2
};

bool isFatalReadError(const int rc)
{
  switch (rc) {
    case LIBUSB_ERROR_NO_DEVICE:
    case LIBUSB_ERROR_ACCESS:
      return true;
    default:
      return false;
  }
}

int readEndpoint(
  libusb_device_handle * handle,
  const uint8_t endpoint,
  std::vector<uint8_t> & out,
  const int timeout_ms,
  int * rc_out)
{
  std::vector<uint8_t> buffer(64U * 1024U, 0U);
  int transferred = 0;
  const int rc = libusb_bulk_transfer(
    handle, endpoint, buffer.data(), static_cast<int>(buffer.size()), &transferred, timeout_ms);
  if (rc_out != nullptr) {
    *rc_out = rc;
  }
  if (rc == 0 && transferred > 0) {
    out.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(transferred));
    return static_cast<int>(EndpointReadState::Ok);
  }
  out.clear();
  if (rc == 0 || rc == LIBUSB_ERROR_TIMEOUT) {
    return static_cast<int>(EndpointReadState::Ok);
  }
  if (isFatalReadError(rc)) {
    return static_cast<int>(EndpointReadState::FatalError);
  }
  return static_cast<int>(EndpointReadState::TransientError);
}

int writeEndpoint(
  libusb_device_handle * handle,
  const uint8_t endpoint,
  const uint8_t * data,
  const int length,
  const int timeout_ms)
{
  int transferred = 0;
  const int rc = libusb_bulk_transfer(
    handle, endpoint, const_cast<unsigned char *>(data), length, &transferred, timeout_ms);
  if (rc != 0) {
    return rc;
  }
  return (transferred == length) ? 0 : LIBUSB_ERROR_IO;
}

}  // namespace

struct VendorProtocol::Impl
{
  libusb_context * ctx{nullptr};
  libusb_device_handle * handle{nullptr};
  std::array<bool, 5> claimed{{false, false, false, false, false}};
  bool open{false};
  int last_read_error_code{0};
  uint8_t last_read_error_endpoint{0};
  int last_write_error_code{0};
  uint8_t last_write_error_endpoint{0};
};

VendorProtocol::VendorProtocol()
: impl_(new Impl())
{
}

VendorProtocol::~VendorProtocol()
{
  close();
  delete impl_;
  impl_ = nullptr;
}

bool VendorProtocol::open()
{
  close();

  if (libusb_init(&impl_->ctx) != 0) {
    return false;
  }
  libusb_set_option(impl_->ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);

  impl_->handle = libusb_open_device_with_vid_pid(impl_->ctx, kVendorId, kProductId);
  if (impl_->handle == nullptr) {
    close();
    return false;
  }

#if defined(__linux__)
  libusb_set_auto_detach_kernel_driver(impl_->handle, 1);
#endif

  for (int if_num = 0; if_num <= 4; ++if_num) {
    const int rc = libusb_claim_interface(impl_->handle, if_num);
    if (rc != 0) {
      close();
      return false;
    }
    impl_->claimed[static_cast<size_t>(if_num)] = true;
  }

  for (int if_num = 1; if_num <= 4; ++if_num) {
    const int rc = libusb_set_interface_alt_setting(impl_->handle, if_num, 1);
    if (rc != 0) {
      close();
      return false;
    }
  }

  impl_->open = true;
  impl_->last_read_error_code = 0;
  impl_->last_read_error_endpoint = 0;
  impl_->last_write_error_code = 0;
  impl_->last_write_error_endpoint = 0;
  return true;
}

void VendorProtocol::close()
{
  if (impl_ == nullptr) {
    return;
  }

  if (impl_->handle != nullptr) {
    for (int if_num = 1; if_num <= 4; ++if_num) {
      (void)libusb_set_interface_alt_setting(impl_->handle, if_num, 0);
    }
    for (int if_num = 0; if_num <= 4; ++if_num) {
      if (impl_->claimed[static_cast<size_t>(if_num)]) {
        (void)libusb_release_interface(impl_->handle, if_num);
        impl_->claimed[static_cast<size_t>(if_num)] = false;
      }
    }
    libusb_close(impl_->handle);
    impl_->handle = nullptr;
  }

  if (impl_->ctx != nullptr) {
    libusb_exit(impl_->ctx);
    impl_->ctx = nullptr;
  }

  impl_->open = false;
}

bool VendorProtocol::isOpen() const
{
  return impl_ != nullptr && impl_->open && impl_->handle != nullptr;
}

bool VendorProtocol::sendCommandFrame(
  const uint32_t command_id,
  const uint32_t arg0,
  const uint32_t arg1)
{
  if (!isOpen()) {
    return false;
  }

  uint8_t frame[20] = {
    0xc0, 0xff, 0xee, 0x00,  // magic
    0x0c, 0x00, 0x00, 0x00,  // payload length = 12
    0x00, 0x00, 0x00, 0x00,  // command
    0x00, 0x00, 0x00, 0x00,  // arg0
    0x00, 0x00, 0x00, 0x00   // arg1
  };

  std::memcpy(frame + 8, &command_id, sizeof(command_id));
  std::memcpy(frame + 12, &arg0, sizeof(arg0));
  std::memcpy(frame + 16, &arg1, sizeof(arg1));

  return sendRawTransfer(
    kCommandOutEndpoint,
    std::vector<uint8_t>(frame, frame + sizeof(frame)),
    200);
}

bool VendorProtocol::sendRawTransfer(
  const uint8_t endpoint,
  const std::vector<uint8_t> & payload,
  const int timeout_ms)
{
  if (!isOpen() || payload.empty()) {
    if (impl_ != nullptr) {
      impl_->last_write_error_code = LIBUSB_ERROR_INVALID_PARAM;
      impl_->last_write_error_endpoint = endpoint;
    }
    return false;
  }
  const int rc = writeEndpoint(
    impl_->handle,
    endpoint,
    payload.data(),
    static_cast<int>(payload.size()),
    std::max(1, timeout_ms));
  impl_->last_write_error_code = rc;
  impl_->last_write_error_endpoint = endpoint;
  return rc == 0;
}

bool VendorProtocol::poll(VendorPollResult & out, const int timeout_ms)
{
  if (!isOpen()) {
    return false;
  }

  out = VendorPollResult{};
  impl_->last_read_error_code = 0;
  impl_->last_read_error_endpoint = 0;

  const int status_ep_timeout_ms = std::max(1, timeout_ms);
  const int stream_ep_timeout_ms = std::max(1, timeout_ms / 2);
  for (size_t i = 0; i < kInputEndpoints.size(); ++i) {
    std::vector<uint8_t> * target = nullptr;
    int ep_timeout = status_ep_timeout_ms;
    switch (i) {
      case 0:
        target = &out.endpoint81;
        ep_timeout = status_ep_timeout_ms;
        break;
      case 1:
        target = &out.endpoint82;
        ep_timeout = status_ep_timeout_ms;
        break;
      case 2:
        target = &out.endpoint83;
        ep_timeout = stream_ep_timeout_ms;
        break;
      case 3:
        target = &out.endpoint84;
        ep_timeout = stream_ep_timeout_ms;
        break;
      default:
        target = &out.endpoint85;
        ep_timeout = stream_ep_timeout_ms;
        break;
    }

    int rc = 0;
    const int state = readEndpoint(
      impl_->handle, kInputEndpoints[i], *target, ep_timeout, &rc);
    if (state == static_cast<int>(EndpointReadState::TransientError)) {
      impl_->last_read_error_code = rc;
      impl_->last_read_error_endpoint = kInputEndpoints[i];
      continue;
    }
    if (state == static_cast<int>(EndpointReadState::FatalError)) {
      impl_->last_read_error_code = rc;
      impl_->last_read_error_endpoint = kInputEndpoints[i];
      // Stream endpoints are optional during startup probing; avoid forcing reconnects
      // only because one of them is not ready yet.
      if (i >= 2U) {
        continue;
      }
      return false;
    }
  }

  return true;
}

int VendorProtocol::lastReadErrorCode() const
{
  if (impl_ == nullptr) {
    return 0;
  }
  return impl_->last_read_error_code;
}

uint8_t VendorProtocol::lastReadErrorEndpoint() const
{
  if (impl_ == nullptr) {
    return 0U;
  }
  return impl_->last_read_error_endpoint;
}

int VendorProtocol::lastWriteErrorCode() const
{
  if (impl_ == nullptr) {
    return 0;
  }
  return impl_->last_write_error_code;
}

uint8_t VendorProtocol::lastWriteErrorEndpoint() const
{
  if (impl_ == nullptr) {
    return 0U;
  }
  return impl_->last_write_error_endpoint;
}

}  // namespace structure_core
