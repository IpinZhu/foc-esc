#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace bridge::v2 {
constexpr std::size_t kHeaderSize = 34;
constexpr std::size_t kMaxPayload = 198;
constexpr std::size_t kMaxPacket = kHeaderSize + kMaxPayload + 2;
constexpr std::uint8_t kVersion = 2;
enum Kind : std::uint8_t {
  hello = 1,
  identity = 2,
  request = 3,
  response = 4,
  telemetry = 5,
  bridge_hello = 6,
  bridge_info = 7
};
struct Packet {
  std::array<std::uint8_t, kMaxPacket> bytes{};
  std::size_t size = 0;
};
struct Frame {
  std::uint8_t kind = 0;
  std::uint64_t session = 0, boot = 0;
  std::uint32_t request = 0, sequence = 0, permit = 0;
  const std::uint8_t *payload = nullptr;
  std::size_t length = 0;
};
inline std::uint16_t crc16(const std::uint8_t *p, std::size_t n) {
  std::uint16_t c = 0xffff;
  for (std::size_t i = 0; i < n; i++) {
    c ^= std::uint16_t(p[i]) << 8;
    for (int b = 0; b < 8; b++)
      c = std::uint16_t((c << 1) ^ ((c & 0x8000) ? 0x1021 : 0));
  }
  return c;
}
template <class T> inline void put(std::uint8_t *p, T v) {
  for (std::size_t i = 0; i < sizeof(T); i++)
    p[i] = std::uint8_t(v >> (8 * i));
}
template <class T> inline T get(const std::uint8_t *p) {
  T v = 0;
  for (std::size_t i = 0; i < sizeof(T); i++)
    v |= T(p[i]) << (8 * i);
  return v;
}
inline bool encode(const Frame &f, Packet &out) {
  out.size = 0;
  if (f.kind < hello || f.kind > bridge_info || f.length > kMaxPayload ||
      (f.length && f.payload == nullptr))
    return false;
  auto *p = out.bytes.data();
  p[0] = 'F';
  p[1] = 'C';
  p[2] = kVersion;
  p[3] = f.kind;
  put<std::uint16_t>(p + 4, std::uint16_t(f.length));
  put(p + 6, f.session);
  put(p + 14, f.boot);
  put(p + 22, f.request);
  put(p + 26, f.sequence);
  put(p + 30, f.permit);
  if (f.length)
    std::memcpy(p + kHeaderSize, f.payload, f.length);
  out.size = kHeaderSize + f.length + 2;
  put(p + out.size - 2, crc16(p, out.size - 2));
  return true;
}
inline bool decode(const std::uint8_t *p, std::size_t n, Frame &f) {
  f = {};
  if (!p || n < kHeaderSize + 2 || n > kMaxPacket || p[0] != 'F' ||
      p[1] != 'C' || p[2] != kVersion || p[3] < hello || p[3] > bridge_info)
    return false;
  auto len = get<std::uint16_t>(p + 4);
  if (len > kMaxPayload || n != kHeaderSize + len + 2 ||
      get<std::uint16_t>(p + n - 2) != crc16(p, n - 2))
    return false;
  f = {p[3],
       get<std::uint64_t>(p + 6),
       get<std::uint64_t>(p + 14),
       get<std::uint32_t>(p + 22),
       get<std::uint32_t>(p + 26),
       get<std::uint32_t>(p + 30),
       p + kHeaderSize,
       len};
  return true;
}
class Decoder {
public:
  bool push(std::uint8_t b, Frame &f) {
    if (n_ == bytes_.size())
      shift();
    bytes_[n_++] = b;
    for (;;) {
      if (!n_)
        return false;
      if (bytes_[0] != 'F') {
        shift();
        continue;
      }
      if (n_ < 2)
        return false;
      if (bytes_[1] != 'C') {
        shift();
        continue;
      }
      if (n_ < 6)
        return false;
      const auto len = get<std::uint16_t>(bytes_.data() + 4);
      if (bytes_[2] != kVersion || bytes_[3] < hello ||
          bytes_[3] > bridge_info || len > kMaxPayload) {
        shift();
        continue;
      }
      const std::size_t total = kHeaderSize + len + 2;
      if (n_ < total)
        return false;
      if (decode(bytes_.data(), total, f)) {
        std::memmove(bytes_.data(), bytes_.data() + total, n_ - total);
        n_ -= total;
        return true;
      }
      shift();
    }
  }
  void reset() { n_ = 0; }

private:
  void shift() { std::memmove(bytes_.data(), bytes_.data() + 1, --n_); }
  std::array<std::uint8_t, kMaxPacket> bytes_{};
  std::size_t n_ = 0;
};
} // namespace bridge::v2
