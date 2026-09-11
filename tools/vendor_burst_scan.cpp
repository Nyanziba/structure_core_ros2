#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#if __has_include(<libusb.h>)
#include <libusb.h>
#elif __has_include(<libusb-1.0/libusb.h>)
#include <libusb-1.0/libusb.h>
#else
#error "libusb headers not found"
#endif

namespace
{

constexpr uint16_t kVendorId = 0x2959;
constexpr uint16_t kProductId = 0x3001;
constexpr std::array<uint8_t, 5> kOutEndpoints = {0x01, 0x02, 0x03, 0x04, 0x05};
constexpr std::array<uint8_t, 5> kInEndpoints = {0x81, 0x82, 0x83, 0x84, 0x85};

struct TransferStep
{
  uint8_t endpoint{0x01};
  std::vector<uint8_t> payload;
  int delay_ms_after{0};
};

struct Candidate
{
  std::string template_name;
  uint8_t primary_ep{0x01};
  uint32_t command_id{0};
  uint32_t arg0{0};
  uint32_t arg1{0};
};

struct Result
{
  Candidate candidate;
  std::array<int, 5> in_bytes{{0, 0, 0, 0, 0}};
  int send_failures{0};
  int last_send_rc{0};
  int fatal_read_rc{0};
  uint8_t fatal_read_ep{0};
  int score{std::numeric_limits<int>::min()};
};

bool isFatalTransferRc(const int rc)
{
  return
    rc == LIBUSB_ERROR_NO_DEVICE ||
    rc == LIBUSB_ERROR_ACCESS ||
    rc == LIBUSB_ERROR_NOT_FOUND;
}

void appendLe32(std::vector<uint8_t> & out, const uint32_t value)
{
  out.push_back(static_cast<uint8_t>(value & 0xffU));
  out.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
  out.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
  out.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
}

std::vector<uint8_t> makeFrame20(const uint32_t cmd, const uint32_t arg0, const uint32_t arg1)
{
  std::vector<uint8_t> out = {
    0xc0U, 0xffU, 0xeeU, 0x00U,
    0x0cU, 0x00U, 0x00U, 0x00U
  };
  appendLe32(out, cmd);
  appendLe32(out, arg0);
  appendLe32(out, arg1);
  return out;
}

std::vector<uint8_t> makePayload12(const uint32_t cmd, const uint32_t arg0, const uint32_t arg1)
{
  std::vector<uint8_t> out;
  appendLe32(out, cmd);
  appendLe32(out, arg0);
  appendLe32(out, arg1);
  return out;
}

std::vector<TransferStep> buildSteps(
  const Candidate & c,
  const std::vector<uint8_t> & frame20,
  const std::vector<uint8_t> & payload12)
{
  const std::vector<uint8_t> header4 = {0xc0U, 0xffU, 0xeeU, 0x00U};
  const std::vector<uint8_t> header8 = {0x00U, 0x00U, 0x00U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U};

  std::vector<TransferStep> steps;
  if (c.template_name == "frame20") {
    steps.push_back({c.primary_ep, frame20, 2});
    return steps;
  }
  if (c.template_name == "payload12") {
    steps.push_back({c.primary_ep, payload12, 2});
    return steps;
  }
  if (c.template_name == "preface_frame20") {
    steps.push_back({c.primary_ep, header4, 1});
    steps.push_back({c.primary_ep, header8, 1});
    steps.push_back({c.primary_ep, frame20, 2});
    return steps;
  }
  if (c.template_name == "preface_payload12") {
    steps.push_back({c.primary_ep, header4, 1});
    steps.push_back({c.primary_ep, header8, 1});
    steps.push_back({c.primary_ep, payload12, 2});
    return steps;
  }
  if (c.template_name == "frame_then_payload") {
    steps.push_back({c.primary_ep, frame20, 1});
    steps.push_back({c.primary_ep, payload12, 2});
    return steps;
  }
  if (c.template_name == "preface_frame_then_payload") {
    steps.push_back({c.primary_ep, header4, 1});
    steps.push_back({c.primary_ep, header8, 1});
    steps.push_back({c.primary_ep, frame20, 1});
    steps.push_back({c.primary_ep, payload12, 2});
    return steps;
  }
  if (c.template_name == "broadcast_frame20") {
    steps.push_back({c.primary_ep, frame20, 1});
    for (const uint8_t ep : kOutEndpoints) {
      if (ep == c.primary_ep) {
        continue;
      }
      steps.push_back({ep, frame20, 1});
    }
    return steps;
  }
  if (c.template_name == "broadcast_payload12") {
    steps.push_back({c.primary_ep, payload12, 1});
    for (const uint8_t ep : kOutEndpoints) {
      if (ep == c.primary_ep) {
        continue;
      }
      steps.push_back({ep, payload12, 1});
    }
    return steps;
  }

  // Fallback
  steps.push_back({c.primary_ep, frame20, 2});
  return steps;
}

class DeviceSession
{
public:
  ~DeviceSession()
  {
    close();
  }

  bool open(const int retries, const int retry_wait_ms)
  {
    close();
    if (libusb_init(&ctx_) != 0) {
      return false;
    }
    libusb_set_option(ctx_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_NONE);

    for (int i = 0; i < retries; ++i) {
      handle_ = libusb_open_device_with_vid_pid(ctx_, kVendorId, kProductId);
      if (handle_ != nullptr) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_wait_ms));
    }
    if (handle_ == nullptr) {
      close();
      return false;
    }

    for (int if_num = 0; if_num <= 4; ++if_num) {
      const int rc = libusb_claim_interface(handle_, if_num);
      if (rc != 0) {
        close();
        return false;
      }
      claimed_[static_cast<size_t>(if_num)] = true;
    }

    for (int if_num = 1; if_num <= 4; ++if_num) {
      const int rc = libusb_set_interface_alt_setting(handle_, if_num, 1);
      if (rc != 0) {
        close();
        return false;
      }
    }

    return true;
  }

  void close()
  {
    if (handle_ != nullptr) {
      for (int if_num = 1; if_num <= 4; ++if_num) {
        (void)libusb_set_interface_alt_setting(handle_, if_num, 0);
      }
      for (int if_num = 0; if_num <= 4; ++if_num) {
        if (claimed_[static_cast<size_t>(if_num)]) {
          (void)libusb_release_interface(handle_, if_num);
          claimed_[static_cast<size_t>(if_num)] = false;
        }
      }
      libusb_close(handle_);
      handle_ = nullptr;
    }
    if (ctx_ != nullptr) {
      libusb_exit(ctx_);
      ctx_ = nullptr;
    }
  }

  bool isOpen() const
  {
    return handle_ != nullptr;
  }

  int send(const uint8_t endpoint, const std::vector<uint8_t> & payload, const int timeout_ms)
  {
    if (!isOpen() || payload.empty()) {
      return LIBUSB_ERROR_INVALID_PARAM;
    }
    int transferred = 0;
    return libusb_bulk_transfer(
      handle_,
      endpoint,
      const_cast<unsigned char *>(payload.data()),
      static_cast<int>(payload.size()),
      &transferred,
      timeout_ms);
  }

  int read(const uint8_t endpoint, int & transferred, const int timeout_ms)
  {
    if (!isOpen()) {
      transferred = 0;
      return LIBUSB_ERROR_NO_DEVICE;
    }
    if (read_buf_.empty()) {
      read_buf_.resize(64U * 1024U);
    }
    return libusb_bulk_transfer(
      handle_,
      endpoint,
      read_buf_.data(),
      static_cast<int>(read_buf_.size()),
      &transferred,
      timeout_ms);
  }

private:
  libusb_context * ctx_{nullptr};
  libusb_device_handle * handle_{nullptr};
  std::array<bool, 5> claimed_{{false, false, false, false, false}};
  std::vector<unsigned char> read_buf_;
};

Result runCandidate(
  DeviceSession & session,
  const Candidate & candidate,
  const int read_window_ms,
  const bool verbose)
{
  Result result;
  result.candidate = candidate;

  const std::vector<uint8_t> frame20 = makeFrame20(candidate.command_id, candidate.arg0, candidate.arg1);
  const std::vector<uint8_t> payload12 = makePayload12(candidate.command_id, candidate.arg0, candidate.arg1);
  const std::vector<TransferStep> steps = buildSteps(candidate, frame20, payload12);

  for (const auto & step : steps) {
    const int rc = session.send(step.endpoint, step.payload, 200);
    if (rc != 0) {
      result.send_failures++;
      result.last_send_rc = rc;
      if (isFatalTransferRc(rc)) {
        result.fatal_read_rc = rc;
        result.fatal_read_ep = step.endpoint;
        break;
      }
    }
    if (step.delay_ms_after > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(step.delay_ms_after));
    }
  }

  if (result.fatal_read_rc == 0) {
    const auto t0 = std::chrono::steady_clock::now();
    while (
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count() < read_window_ms)
    {
      for (size_t i = 0; i < kInEndpoints.size(); ++i) {
        int got = 0;
        const int rc = session.read(kInEndpoints[i], got, 8);
        if (rc == 0 && got > 0) {
          result.in_bytes[i] += got;
          continue;
        }
        if (rc == LIBUSB_ERROR_TIMEOUT || rc == 0) {
          continue;
        }
        if (isFatalTransferRc(rc)) {
          result.fatal_read_rc = rc;
          result.fatal_read_ep = kInEndpoints[i];
          break;
        }
      }
      if (result.fatal_read_rc != 0) {
        break;
      }
    }
  }

  const int status = result.in_bytes[0] + result.in_bytes[1];
  const int stream = result.in_bytes[2] + result.in_bytes[3] + result.in_bytes[4];
  result.score = (stream * 100) + status - (result.send_failures * 2000);
  if (result.fatal_read_rc != 0) {
    result.score -= 10000;
  }

  if (verbose || stream > 0) {
    std::cout
      << "template=" << candidate.template_name
      << " ep=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(candidate.primary_ep)
      << std::dec
      << " cmd=0x" << std::hex << candidate.command_id
      << " arg0=0x" << candidate.arg0
      << " arg1=0x" << candidate.arg1
      << std::dec
      << " bytes[81..85]=[" << result.in_bytes[0] << "," << result.in_bytes[1] << ","
      << result.in_bytes[2] << "," << result.in_bytes[3] << "," << result.in_bytes[4] << "]"
      << " send_fail=" << result.send_failures
      << " send_rc=" << result.last_send_rc
      << " fatal_ep=0x" << std::hex << static_cast<int>(result.fatal_read_ep)
      << std::dec
      << " fatal_rc=" << result.fatal_read_rc
      << " score=" << result.score
      << "\n";
  }

  return result;
}

void keepTopResults(std::vector<Result> & top, const Result & result, const size_t top_n)
{
  top.push_back(result);
  std::sort(top.begin(), top.end(), [](const Result & a, const Result & b) {
    return a.score > b.score;
  });
  if (top.size() > top_n) {
    top.resize(top_n);
  }
}

std::vector<Candidate> buildCandidates()
{
  const std::array<std::string, 8> templates = {
    "frame20",
    "payload12",
    "preface_frame20",
    "preface_payload12",
    "frame_then_payload",
    "preface_frame_then_payload",
    "broadcast_frame20",
    "broadcast_payload12"
  };
  const std::array<uint32_t, 7> cmds = {
    0x10000013U, 0x10000010U, 0x10000011U, 0x10000012U, 0x10000014U, 0x10000015U, 0x10000016U
  };
  const std::array<uint32_t, 6> arg0s = {0x0fU, 0x01U, 0x03U, 0x07U, 0x08U, 0x00U};

  std::vector<Candidate> candidates;
  candidates.reserve(templates.size() * cmds.size() * arg0s.size() * kOutEndpoints.size());
  for (const auto cmd : cmds) {
    for (const auto arg0 : arg0s) {
      for (const auto ep : kOutEndpoints) {
        for (const auto & t : templates) {
          Candidate c;
          c.template_name = t;
          c.primary_ep = ep;
          c.command_id = cmd;
          c.arg0 = arg0;
          c.arg1 = 0x00000000U;
          candidates.push_back(c);
        }
      }
    }
  }
  return candidates;
}

}  // namespace

int main(int argc, char ** argv)
{
  int max_tests = 600;
  int read_window_ms = 140;
  int open_retries = 8;
  int open_wait_ms = 40;
  int max_consecutive_open_failures = 16;
  bool verbose = false;
  std::cout.setf(std::ios::unitbuf);

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--max-tests" && i + 1 < argc) {
      max_tests = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--read-ms" && i + 1 < argc) {
      read_window_ms = std::max(20, std::atoi(argv[++i]));
    } else if (arg == "--open-retries" && i + 1 < argc) {
      open_retries = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--open-wait-ms" && i + 1 < argc) {
      open_wait_ms = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--max-open-fails" && i + 1 < argc) {
      max_consecutive_open_failures = std::max(1, std::atoi(argv[++i]));
    } else if (arg == "--verbose") {
      verbose = true;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      std::cerr << "usage: vendor_burst_scan [--max-tests N] [--read-ms M] [--open-retries N] "
                   "[--open-wait-ms M] [--max-open-fails N] [--verbose]\n";
      return 2;
    }
  }

  std::vector<Candidate> candidates = buildCandidates();
  if (max_tests < static_cast<int>(candidates.size())) {
    candidates.resize(static_cast<size_t>(max_tests));
  }

  DeviceSession session;
  if (!session.open(open_retries, open_wait_ms)) {
    std::cerr << "Failed to open/claim device 0x2959:0x3001\n";
    return 3;
  }

  std::vector<Result> top;
  top.reserve(20);

  int tested = 0;
  int disconnected = 0;
  int stream_hits = 0;
  int open_failures = 0;
  int consecutive_open_failures = 0;

  for (const auto & c : candidates) {
    if (!session.isOpen()) {
      if (!session.open(open_retries, open_wait_ms)) {
        open_failures++;
        consecutive_open_failures++;
        if (consecutive_open_failures >= max_consecutive_open_failures) {
          std::cout
            << "abort: reached max consecutive open failures ("
            << max_consecutive_open_failures << ")\n";
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        continue;
      }
      consecutive_open_failures = 0;
    }

    const Result r = runCandidate(session, c, read_window_ms, verbose);
    tested++;
    keepTopResults(top, r, 20);

    const int stream = r.in_bytes[2] + r.in_bytes[3] + r.in_bytes[4];
    if (stream > 0) {
      stream_hits++;
      std::cout << "STREAM_HIT ";
      runCandidate(session, c, 60, true);
    }

    if (isFatalTransferRc(r.fatal_read_rc)) {
      disconnected++;
      session.close();
      std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }

    if ((tested % 50) == 0) {
      std::cout
        << "progress tested=" << tested
        << " stream_hits=" << stream_hits
        << " disconnects=" << disconnected
        << " open_failures=" << open_failures
        << "\n";
    }
  }

  std::cout << "\n=== SUMMARY ===\n";
  std::cout << "tested=" << tested
            << " stream_hits=" << stream_hits
            << " disconnects=" << disconnected
            << " open_failures=" << open_failures
            << " read_window_ms=" << read_window_ms
            << "\n";
  std::cout << "Top candidates:\n";
  for (const auto & r : top) {
    std::cout
      << " score=" << std::setw(8) << r.score
      << " template=" << std::setw(24) << std::left << r.candidate.template_name << std::right
      << " ep=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(r.candidate.primary_ep)
      << std::dec << std::setfill(' ')
      << " cmd=0x" << std::hex << r.candidate.command_id
      << " arg0=0x" << r.candidate.arg0
      << std::dec
      << " bytes[81..85]=[" << r.in_bytes[0] << "," << r.in_bytes[1] << ","
      << r.in_bytes[2] << "," << r.in_bytes[3] << "," << r.in_bytes[4] << "]"
      << " fatal(ep=0x" << std::hex << static_cast<int>(r.fatal_read_ep)
      << ",rc=" << std::dec << r.fatal_read_rc << ")"
      << "\n";
  }

  return 0;
}
