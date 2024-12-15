#include "frc846/base/compression.h"

#include <cctype>
#include <cstring>
#include <iostream>
#include <string>

namespace frc846::base {

std::pair<uint8_t, bool> Compression::char_conv(char x) {
  uint8_t val = 0;
  bool is_upper = (x >= 'A' && x <= 'Z');

  x = std::tolower(x);

  const std::string ENCODING =
      "\nabcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*()[]<>|;:',./?~_- ";

  size_t pos = ENCODING.find(x);
  if (pos != std::string::npos) {
    val = pos;
  } else {
    val = ENCODING.size() - 1;
  }

  return std::make_pair(val, is_upper);
}

std::vector<uint8_t> Compression::pack_bytes(std::vector<uint8_t> conv) {
  std::vector<uint8_t> output;
  uint32_t buffer = 0;
  int bitCount = 0;

  for (uint8_t value : conv) {
    buffer |= (value & 0x3F) << (18 - bitCount);
    bitCount += 6;

    while (bitCount >= 8) {
      output.push_back((buffer >> 16) & 0xFF);
      buffer <<= 8;
      bitCount -= 8;
    }
  }

  if (bitCount > 0) {
    output.push_back((buffer >> 16) & 0xFF);
  }

  return output;
}

std::vector<uint8_t> Compression::compress(const std::string& data) {
  std::vector<uint8_t> conv;
  for (char c : data) {
    auto [val, is_upper] = char_conv(c);
    if (is_upper) {
      conv.push_back((uint8_t)63);
    }
    conv.push_back(val);
  }
  std::vector<uint8_t> packed = pack_bytes(conv);

  return packed;
}

}  // namespace frc846::base