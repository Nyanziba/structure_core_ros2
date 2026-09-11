#include "structure_core/uvc_capture.hpp"

namespace structure_core
{

struct UvcCaptureImpl
{
};

UvcCapture::UvcCapture()
: impl_(new UvcCaptureImpl())
{
}

UvcCapture::~UvcCapture() = default;

bool UvcCapture::open(
  const std::string & name_filter,
  const int match_index,
  const int requested_width,
  const int requested_height,
  const double requested_fps)
{
  (void)name_filter;
  (void)match_index;
  (void)requested_width;
  (void)requested_height;
  (void)requested_fps;
  return false;
}

void UvcCapture::close()
{
}

bool UvcCapture::isOpen() const
{
  return false;
}

std::string UvcCapture::deviceName() const
{
  return {};
}

bool UvcCapture::getLatestFrame(UvcFrame & out_frame) const
{
  (void)out_frame;
  return false;
}

std::vector<std::string> UvcCapture::listDevices()
{
  return {};
}

}  // namespace structure_core
