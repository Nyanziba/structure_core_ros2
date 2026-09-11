#ifndef STRUCTURE_CORE__VENDOR_PROTOCOL_HPP_
#define STRUCTURE_CORE__VENDOR_PROTOCOL_HPP_

#include <cstdint>
#include <vector>

namespace structure_core
{

struct VendorPollResult
{
  std::vector<uint8_t> endpoint81;
  std::vector<uint8_t> endpoint82;
  std::vector<uint8_t> endpoint83;
  std::vector<uint8_t> endpoint84;
  std::vector<uint8_t> endpoint85;

  bool hasAnyStreamData() const
  {
    return !endpoint83.empty() || !endpoint84.empty() || !endpoint85.empty();
  }

  bool hasAnyStatusData() const
  {
    return !endpoint81.empty() || !endpoint82.empty();
  }
};

class VendorProtocol
{
public:
  VendorProtocol();
  ~VendorProtocol();

  VendorProtocol(const VendorProtocol &) = delete;
  VendorProtocol & operator=(const VendorProtocol &) = delete;

  bool open();
  void close();
  bool isOpen() const;

  bool sendCommandFrame(uint32_t command_id, uint32_t arg0, uint32_t arg1);
  bool sendRawTransfer(uint8_t endpoint, const std::vector<uint8_t> & payload, int timeout_ms);
  bool poll(VendorPollResult & out, int timeout_ms);
  int lastReadErrorCode() const;
  uint8_t lastReadErrorEndpoint() const;
  int lastWriteErrorCode() const;
  uint8_t lastWriteErrorEndpoint() const;

private:
  struct Impl;
  Impl * impl_;
};

}  // namespace structure_core

#endif  // STRUCTURE_CORE__VENDOR_PROTOCOL_HPP_
