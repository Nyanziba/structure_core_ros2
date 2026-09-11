#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "structure_core/uvc_capture.hpp"

namespace structure_core
{

struct UvcCaptureImpl
{
  mutable std::mutex lock;
  UvcFrame latest_frame;
  bool has_frame{false};
  bool is_open{false};
  std::string device_name;
  AVCaptureSession * session{nil};
  AVCaptureVideoDataOutput * output{nil};
  AVCaptureDeviceInput * input{nil};
  dispatch_queue_t queue{nil};
  id delegate{nil};
};

}  // namespace structure_core

@interface StructureCoreFrameDelegate : NSObject<AVCaptureVideoDataOutputSampleBufferDelegate>
@property(nonatomic, assign) void * impl_ptr;
@end

@implementation StructureCoreFrameDelegate
- (void)captureOutput:(AVCaptureOutput *)output didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
  fromConnection:(AVCaptureConnection *)connection
{
  (void)output;
  (void)connection;

  auto * impl = static_cast<structure_core::UvcCaptureImpl *>(self.impl_ptr);
  if (impl == nullptr) {
    return;
  }

  CVImageBufferRef image = CMSampleBufferGetImageBuffer(sampleBuffer);
  if (image == nullptr) {
    return;
  }

  CVPixelBufferLockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
  const auto width = static_cast<int>(CVPixelBufferGetWidth(image));
  const auto height = static_cast<int>(CVPixelBufferGetHeight(image));
  const auto bytes_per_row = static_cast<int>(CVPixelBufferGetBytesPerRow(image));
  const OSType format = CVPixelBufferGetPixelFormatType(image);
  const uint8_t * base = static_cast<const uint8_t *>(CVPixelBufferGetBaseAddress(image));

  if (base == nullptr || width <= 0 || height <= 0 || format != kCVPixelFormatType_32BGRA) {
    CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);
    return;
  }

  std::vector<uint8_t> rgb(
    static_cast<size_t>(width) * static_cast<size_t>(height) * static_cast<size_t>(3), 0U);

  for (int y = 0; y < height; ++y) {
    const uint8_t * row = base + (static_cast<size_t>(y) * static_cast<size_t>(bytes_per_row));
    for (int x = 0; x < width; ++x) {
      const size_t dst_idx =
        (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 3U;
      const size_t src_idx = static_cast<size_t>(x) * 4U;
      // BGRA -> RGB
      rgb[dst_idx + 0U] = row[src_idx + 2U];
      rgb[dst_idx + 1U] = row[src_idx + 1U];
      rgb[dst_idx + 2U] = row[src_idx + 0U];
    }
  }
  CVPixelBufferUnlockBaseAddress(image, kCVPixelBufferLock_ReadOnly);

  uint64_t capture_time_ns = 0U;
  const CMTime pts = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
  if (CMTIME_IS_NUMERIC(pts)) {
    const double sec = CMTimeGetSeconds(pts);
    if (std::isfinite(sec) && sec >= 0.0) {
      capture_time_ns = static_cast<uint64_t>(sec * 1e9);
    }
  }

  structure_core::UvcFrame frame;
  frame.width = width;
  frame.height = height;
  frame.encoding = "rgb8";
  frame.capture_time_ns = capture_time_ns;
  frame.data = std::move(rgb);

  std::lock_guard<std::mutex> guard(impl->lock);
  impl->latest_frame = std::move(frame);
  impl->has_frame = true;
}
@end

namespace
{

std::string toStdString(NSString * value)
{
  if (value == nil) {
    return {};
  }
  const char * utf8 = [value UTF8String];
  if (utf8 == nullptr) {
    return {};
  }
  return std::string(utf8);
}

std::string toLowerAscii(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char c) {return static_cast<char>(std::tolower(c));});
  return value;
}

NSArray<AVCaptureDevice *> * videoDevices()
{
  if (@available(macOS 10.15, *)) {
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wdeprecated-declarations"
    AVCaptureDeviceDiscoverySession * session =
      [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[
      AVCaptureDeviceTypeExternalUnknown,
      AVCaptureDeviceTypeBuiltInWideAngleCamera
    ]
      mediaType:AVMediaTypeVideo
      position:AVCaptureDevicePositionUnspecified];
    #pragma clang diagnostic pop
    return session.devices;
  }

  return @[];
}

AVCaptureDevice * selectDevice(const std::string & name_filter, const int match_index)
{
  NSArray<AVCaptureDevice *> * devices = videoDevices();
  if (devices.count == 0) {
    return nil;
  }

  NSString * filter = [NSString stringWithUTF8String:name_filter.c_str()];
  const std::string lowered = toLowerAscii(name_filter);
  const bool use_filter =
    !name_filter.empty() && name_filter != "*" &&
    lowered != "any" && lowered != "all";
  if (!use_filter) {
    filter = @"";
  }
  int current_match_index = 0;
  for (AVCaptureDevice * device in devices) {
    const NSString * name = (device.localizedName != nil) ? device.localizedName : @"";
    if (use_filter && filter.length > 0) {
      const NSRange hit = [name rangeOfString:filter options:NSCaseInsensitiveSearch];
      if (hit.location == NSNotFound) {
        continue;
      }
    }
    if (current_match_index == match_index) {
      return device;
    }
    ++current_match_index;
  }
  return nil;
}

void tryConfigureFormat(
  AVCaptureDevice * device,
  const int requested_width,
  const int requested_height,
  const double requested_fps)
{
  if (requested_width <= 0 || requested_height <= 0 || requested_fps <= 0.0 || device == nil) {
    return;
  }

  NSError * error = nil;
  if (![device lockForConfiguration:&error]) {
    return;
  }

  AVCaptureDeviceFormat * best = nil;
  int best_score = std::numeric_limits<int>::max();

  for (AVCaptureDeviceFormat * format in device.formats) {
    const CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
    if (dims.width <= 0 || dims.height <= 0) {
      continue;
    }

    bool fps_supported = false;
    for (AVFrameRateRange * range in format.videoSupportedFrameRateRanges) {
      if (range.minFrameRate <= requested_fps && requested_fps <= range.maxFrameRate) {
        fps_supported = true;
        break;
      }
    }

    int score = std::abs(dims.width - requested_width) + std::abs(dims.height - requested_height);
    if (!fps_supported) {
      score += 1000000;
    }

    if (score < best_score) {
      best_score = score;
      best = format;
    }
  }

  if (best != nil) {
    device.activeFormat = best;

    for (AVFrameRateRange * range in best.videoSupportedFrameRateRanges) {
      if (range.minFrameRate <= requested_fps && requested_fps <= range.maxFrameRate) {
        const auto rounded_fps = static_cast<int32_t>(std::lround(requested_fps));
        if (rounded_fps > 0) {
          const CMTime frame_duration = CMTimeMake(1, rounded_fps);
          device.activeVideoMinFrameDuration = frame_duration;
          device.activeVideoMaxFrameDuration = frame_duration;
        }
        break;
      }
    }
  }

  [device unlockForConfiguration];
}

}  // namespace

namespace structure_core
{

UvcCapture::UvcCapture()
: impl_(new UvcCaptureImpl())
{
}

UvcCapture::~UvcCapture()
{
  close();
}

bool UvcCapture::open(
  const std::string & name_filter,
  const int match_index,
  const int requested_width,
  const int requested_height,
  const double requested_fps)
{
  close();

  if (match_index < 0) {
    return false;
  }

  @autoreleasepool {
    AVCaptureDevice * device = selectDevice(name_filter, match_index);
    if (device == nil) {
      return false;
    }

    tryConfigureFormat(device, requested_width, requested_height, requested_fps);

    NSError * input_error = nil;
    impl_->input = [AVCaptureDeviceInput deviceInputWithDevice:device error:&input_error];
    if (impl_->input == nil) {
      return false;
    }

    impl_->session = [[AVCaptureSession alloc] init];
    [impl_->session beginConfiguration];

    if ([impl_->session canAddInput:impl_->input]) {
      [impl_->session addInput:impl_->input];
    } else {
      [impl_->session commitConfiguration];
      close();
      return false;
    }

    impl_->output = [[AVCaptureVideoDataOutput alloc] init];
    impl_->output.alwaysDiscardsLateVideoFrames = YES;
    impl_->output.videoSettings = @{
      (__bridge NSString *)kCVPixelBufferPixelFormatTypeKey: @(kCVPixelFormatType_32BGRA)
    };

    impl_->queue = dispatch_queue_create("structure_core.uvc_capture", DISPATCH_QUEUE_SERIAL);
    StructureCoreFrameDelegate * delegate = [[StructureCoreFrameDelegate alloc] init];
    delegate.impl_ptr = impl_.get();
    impl_->delegate = delegate;

    [impl_->output setSampleBufferDelegate:delegate queue:impl_->queue];
    if ([impl_->session canAddOutput:impl_->output]) {
      [impl_->session addOutput:impl_->output];
    } else {
      [impl_->session commitConfiguration];
      close();
      return false;
    }

    [impl_->session commitConfiguration];
    [impl_->session startRunning];

    impl_->device_name = toStdString(device.localizedName);
    impl_->is_open = impl_->session.isRunning;
  }

  return impl_->is_open;
}

void UvcCapture::close()
{
  @autoreleasepool {
    if (impl_->output != nil && impl_->delegate != nil) {
      [impl_->output setSampleBufferDelegate:nil queue:nil];
    }

    if (impl_->session != nil && impl_->session.isRunning) {
      [impl_->session stopRunning];
    }

    impl_->delegate = nil;
    impl_->queue = nil;
    impl_->output = nil;
    impl_->input = nil;
    impl_->session = nil;

    {
      std::lock_guard<std::mutex> guard(impl_->lock);
      impl_->latest_frame = UvcFrame{};
      impl_->has_frame = false;
    }

    impl_->is_open = false;
    impl_->device_name.clear();
  }
}

bool UvcCapture::isOpen() const
{
  return impl_->is_open;
}

std::string UvcCapture::deviceName() const
{
  return impl_->device_name;
}

bool UvcCapture::getLatestFrame(UvcFrame & out_frame) const
{
  std::lock_guard<std::mutex> guard(impl_->lock);
  if (!impl_->is_open || !impl_->has_frame) {
    return false;
  }
  out_frame = impl_->latest_frame;
  return out_frame.valid();
}

std::vector<std::string> UvcCapture::listDevices()
{
  std::vector<std::string> devices_out;
  @autoreleasepool {
    NSArray<AVCaptureDevice *> * devices = videoDevices();
    devices_out.reserve(static_cast<size_t>(devices.count));
    for (AVCaptureDevice * device in devices) {
      devices_out.emplace_back(toStdString(device.localizedName));
    }
  }
  return devices_out;
}

}  // namespace structure_core
