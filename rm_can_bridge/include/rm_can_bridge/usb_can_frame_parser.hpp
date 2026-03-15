#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace rm_can_bridge
{

enum class UsbCanParseAction
{
  Parsed,
  SkipByte,
  NeedMoreData,
};

struct ParsedUsbCanFrame
{
  uint32_t can_id{0};
  std::array<uint8_t, 8> payload{};
  size_t consumed_bytes{0};
};

struct UsbCanParseResult
{
  UsbCanParseAction action{UsbCanParseAction::NeedMoreData};
  ParsedUsbCanFrame frame{};
};

namespace detail
{

constexpr size_t kLegacyUsbCanFrameSize = 30;
constexpr uint8_t kLegacyHeader0 = 0x55;
constexpr uint8_t kLegacyHeader1 = 0xAA;
constexpr uint8_t kLegacyFrameLen = 0x1E;
constexpr uint8_t kLegacyTail = 0x88;

constexpr size_t kNewAdapterFrameSize = 16;
constexpr uint8_t kNewAdapterHeader = 0xAA;
constexpr uint8_t kNewAdapterPayloadLen = 0x08;
constexpr uint8_t kNewAdapterTail = 0x55;

inline uint32_t readUint32LE(const std::vector<uint8_t> & buffer, const size_t offset)
{
  return static_cast<uint32_t>(buffer[offset]) |
         (static_cast<uint32_t>(buffer[offset + 1]) << 8) |
         (static_cast<uint32_t>(buffer[offset + 2]) << 16) |
         (static_cast<uint32_t>(buffer[offset + 3]) << 24);
}

inline UsbCanParseResult parseLegacyUsbCanFrame(const std::vector<uint8_t> & buffer)
{
  if (buffer.size() < 2) {
    return {UsbCanParseAction::NeedMoreData, {}};
  }
  if (buffer[1] != kLegacyHeader1) {
    return {UsbCanParseAction::SkipByte, {}};
  }
  if (buffer.size() < 3) {
    return {UsbCanParseAction::NeedMoreData, {}};
  }
  if (buffer[2] != kLegacyFrameLen) {
    return {UsbCanParseAction::SkipByte, {}};
  }
  if (buffer.size() < kLegacyUsbCanFrameSize) {
    return {UsbCanParseAction::NeedMoreData, {}};
  }
  if (buffer[kLegacyUsbCanFrameSize - 1] != kLegacyTail) {
    return {UsbCanParseAction::SkipByte, {}};
  }

  ParsedUsbCanFrame frame{};
  frame.can_id = readUint32LE(buffer, 13);
  for (size_t index = 0; index < frame.payload.size(); ++index) {
    frame.payload[index] = buffer[21 + index];
  }
  frame.consumed_bytes = kLegacyUsbCanFrameSize;
  return {UsbCanParseAction::Parsed, frame};
}

inline UsbCanParseResult parseNewAdapterUsbCanFrame(const std::vector<uint8_t> & buffer)
{
  if (buffer.size() < 3) {
    return {UsbCanParseAction::NeedMoreData, {}};
  }
  if (buffer[2] != kNewAdapterPayloadLen) {
    return {UsbCanParseAction::SkipByte, {}};
  }
  if (buffer.size() < kNewAdapterFrameSize) {
    return {UsbCanParseAction::NeedMoreData, {}};
  }
  if (buffer[kNewAdapterFrameSize - 1] != kNewAdapterTail) {
    return {UsbCanParseAction::SkipByte, {}};
  }

  ParsedUsbCanFrame frame{};
  frame.can_id = readUint32LE(buffer, 3);
  for (size_t index = 0; index < frame.payload.size(); ++index) {
    frame.payload[index] = buffer[7 + index];
  }
  frame.consumed_bytes = kNewAdapterFrameSize;
  return {UsbCanParseAction::Parsed, frame};
}

}  // namespace detail

inline UsbCanParseResult tryParseUsbCanFrame(const std::vector<uint8_t> & buffer)
{
  if (buffer.empty()) {
    return {UsbCanParseAction::NeedMoreData, {}};
  }

  if (buffer[0] == detail::kLegacyHeader0) {
    return detail::parseLegacyUsbCanFrame(buffer);
  }

  if (buffer[0] == detail::kNewAdapterHeader) {
    return detail::parseNewAdapterUsbCanFrame(buffer);
  }

  return {UsbCanParseAction::SkipByte, {}};
}

}  // namespace rm_can_bridge
