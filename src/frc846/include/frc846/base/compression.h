#pragma once

#include <string>
#include <vector>

namespace frc846::base {
class Compression {
 private:
  /*
  @param x input character
  @return pair of converted 6 bit value (stored in 8 bit type) of a character
  and character is uppercase.
  */
  static std::pair<uint8_t, bool> char_conv(char x);

  /*
  Packs 6-bit values into bytes.
  */
  static std::vector<uint8_t> pack_bytes(std::vector<uint8_t> conv);

 public:
  /*
  Compresses a string into a vector of bytes. Reduces string size by
  approximately 25%. Requires matching decrompression function.
  @param data input string
  @return compressed data, 6-bit binary packed into bytes.
  */
  static std::vector<uint8_t> compress(const std::string& data);
};
}  // namespace frc846::base