#ifndef STRUCTURE_CORE__UVC_CAPTURE_HPP_
#define STRUCTURE_CORE__UVC_CAPTURE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace structure_core
{

struct UvcFrame
{
  int width{0};
  int height{0};
  std::string encoding{"rgb8"};
  std::vector<uint8_t> data;
  uint64_t capture_time_ns{0U};

  bool valid() const
  {
    return width > 0 && height > 0 && !data.empty();
  }
};

struct UvcCaptureImpl;

class UvcCapture
{
public:
  UvcCapture();
  ~UvcCapture();

  UvcCapture(const UvcCapture &) = delete;
  UvcCapture & operator=(const UvcCapture &) = delete;

  bool open(
    const std::string & name_filter,
    int match_index,
    int requested_width,
    int requested_height,
    double requested_fps);

  void close();
  bool isOpen() const;
  std::string deviceName() const;
  bool getLatestFrame(UvcFrame & out_frame) const;

  static std::vector<std::string> listDevices();

private:
  std::unique_ptr<UvcCaptureImpl> impl_;
};

}  // namespace structure_core

#endif  // STRUCTURE_CORE__UVC_CAPTURE_HPP_
