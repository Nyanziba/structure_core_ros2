#include "structure_core/vendor_protocol.hpp"

namespace structure_core
{

struct VendorProtocol::Impl
{
};

VendorProtocol::VendorProtocol()
: impl_(new Impl())
{
}

VendorProtocol::~VendorProtocol()
{
  delete impl_;
  impl_ = nullptr;
}

bool VendorProtocol::open()
{
  return false;
}

void VendorProtocol::close()
{
}

bool VendorProtocol::isOpen() const
{
  return false;
}

bool VendorProtocol::sendCommandFrame(const uint32_t command_id, const uint32_t arg0, const uint32_t arg1)
{
  (void)command_id;
  (void)arg0;
  (void)arg1;
  return false;
}

bool VendorProtocol::sendRawTransfer(
  const uint8_t endpoint,
  const std::vector<uint8_t> & payload,
  const int timeout_ms)
{
  (void)endpoint;
  (void)payload;
  (void)timeout_ms;
  return false;
}

bool VendorProtocol::poll(VendorPollResult & out, const int timeout_ms)
{
  (void)timeout_ms;
  out = VendorPollResult{};
  return false;
}

int VendorProtocol::lastReadErrorCode() const
{
  return 0;
}

uint8_t VendorProtocol::lastReadErrorEndpoint() const
{
  return 0U;
}

int VendorProtocol::lastWriteErrorCode() const
{
  return 0;
}

uint8_t VendorProtocol::lastWriteErrorEndpoint() const
{
  return 0U;
}

}  // namespace structure_core
