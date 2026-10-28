#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace bridge {

using Mac = std::array<std::uint8_t, 6>;
constexpr std::size_t kHeaderSize = 12;
constexpr std::size_t kMaxPayload = 224;
constexpr std::size_t kMaxPacket = kHeaderSize + kMaxPayload + 2;
constexpr std::uint8_t kVersion = 1;

struct Packet {
    std::array<std::uint8_t, kMaxPacket> bytes{};
    std::size_t size = 0;
};

struct Frame {
    std::uint32_t session = 0;
    std::uint32_t sequence = 0;
    const std::uint8_t* payload = nullptr;
    std::size_t length = 0;
};

inline std::uint16_t crc16(const std::uint8_t* data, std::size_t size) {
    std::uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint16_t>(data[i]) << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
        }
    }
    return crc;
}

inline void put_u32(std::uint8_t* out, std::uint32_t value) {
    for (unsigned i = 0; i < 4; ++i) {
        out[i] = static_cast<std::uint8_t>(value >> (8 * i));
    }
}

inline std::uint32_t get_u32(const std::uint8_t* data) {
    std::uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i) {
        value |= static_cast<std::uint32_t>(data[i]) << (8 * i);
    }
    return value;
}

inline void seal(Packet& packet) {
    const auto crc = crc16(packet.bytes.data(), packet.size - 2);
    packet.bytes[packet.size - 2] = static_cast<std::uint8_t>(crc);
    packet.bytes[packet.size - 1] = static_cast<std::uint8_t>(crc >> 8);
}

inline bool encode(std::uint32_t session, std::uint32_t sequence,
                   const std::uint8_t* payload, std::size_t length, Packet& packet) {
    packet.size = 0;
    if (payload == nullptr || length == 0 || length > kMaxPayload) {
        return false;
    }
    auto* bytes = packet.bytes.data();
    bytes[0] = 'F';
    bytes[1] = 'C';
    bytes[2] = kVersion;
    bytes[3] = static_cast<std::uint8_t>(length);
    put_u32(bytes + 4, session);
    put_u32(bytes + 8, sequence);
    std::memcpy(bytes + kHeaderSize, payload, length);
    packet.size = kHeaderSize + length + 2;
    seal(packet);
    return true;
}

inline bool decode(const std::uint8_t* bytes, std::size_t size, Frame& frame) {
    frame = {};
    if (bytes == nullptr || size < kHeaderSize + 3 || size > kMaxPacket) {
        return false;
    }
    const std::size_t length = bytes[3];
    if (bytes[0] != 'F' || bytes[1] != 'C' || bytes[2] != kVersion ||
        length == 0 || length > kMaxPayload || size != kHeaderSize + length + 2) {
        return false;
    }
    const auto checksum = static_cast<std::uint16_t>(
        bytes[size - 2] | (static_cast<std::uint16_t>(bytes[size - 1]) << 8));
    if (checksum != crc16(bytes, size - 2)) {
        return false;
    }
    frame = {get_u32(bytes + 4), get_u32(bytes + 8), bytes + kHeaderSize, length};
    return true;
}

inline bool complete_line(const std::uint8_t* bytes, std::size_t size) {
    if (bytes == nullptr || size < 3 || size > kMaxPayload ||
        bytes[size - 2] != '\r' || bytes[size - 1] != '\n') {
        return false;
    }
    for (std::size_t i = 0; i + 2 < size; ++i) {
        if (bytes[i] < 0x20 || bytes[i] > 0x7e) {
            return false;
        }
    }
    return true;
}

inline bool parse_mac(const char* text, Mac& mac) {
    if (text == nullptr || std::strlen(text) != 17) {
        return false;
    }
    Mac parsed{};
    for (std::size_t i = 0; i < parsed.size(); ++i) {
        unsigned value = 0;
        for (unsigned nibble = 0; nibble < 2; ++nibble) {
            const char c = text[i * 3 + nibble];
            unsigned digit;
            if (c >= '0' && c <= '9') {
                digit = static_cast<unsigned>(c - '0');
            } else if (c >= 'a' && c <= 'f') {
                digit = static_cast<unsigned>(c - 'a') + 10;
            } else if (c >= 'A' && c <= 'F') {
                digit = static_cast<unsigned>(c - 'A') + 10;
            } else {
                return false;
            }
            value = value * 16 + digit;
        }
        parsed[i] = static_cast<std::uint8_t>(value);
        if (i < 5 && text[i * 3 + 2] != ':') {
            return false;
        }
    }
    mac = parsed;
    return true;
}

inline bool unicast_mac(const Mac& mac) {
    const Mac zero{};
    return mac != zero && (mac[0] & 1) == 0;
}

} // namespace bridge
