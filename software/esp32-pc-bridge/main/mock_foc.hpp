#pragma once

#include "bridge_protocol.hpp"
#include <cstdio>

namespace bridge::mock {

constexpr std::uint32_t kStepMs = 50;
constexpr std::uint32_t kStepsPerCycle = 8;

inline bool telemetry(std::uint32_t index, std::array<std::uint8_t, kMaxPayload>& out,
                      std::size_t& length) {
    const int written = std::snprintf(
        reinterpret_cast<char*>(out.data()), out.size(),
        "seq=%lu state=1 faults=00000000 ia=1.25 ib=-0.50 ic=-0.75 "
        "id=0.00 iq=1.50 vbus=24.00 vel=12.50 temp=35.0\r\n",
        static_cast<unsigned long>(index * 100u));
    if (written <= 0 || static_cast<std::size_t>(written) >= out.size()) {
        return false;
    }
    length = static_cast<std::size_t>(written);
    return true;
}

inline Packet scenario(std::uint32_t session, std::uint32_t step) {
    const auto cycle = step / kStepsPerCycle;
    const auto stage = step % kStepsPerCycle;
    const auto base_sequence = cycle * 2u;
    std::array<std::uint8_t, kMaxPayload> line{};
    std::size_t length = 0;
    telemetry(base_sequence + (stage == 7 ? 1u : 0u), line, length);
    Packet packet;
    encode(session, base_sequence, line.data(), length, packet);
    switch (stage) {
    case 0:
    case 1:
        break;
    case 2:
        packet.bytes[kHeaderSize] ^= 1;
        break;
    case 3:
        ++packet.bytes[3];
        seal(packet);
        break;
    case 4:
        put_u32(packet.bytes.data() + 8, base_sequence - 1u);
        seal(packet);
        break;
    case 5:
        packet.bytes[0] = '?';
        seal(packet);
        break;
    case 6:
        packet.bytes[packet.size - 3] = '!';
        put_u32(packet.bytes.data() + 8, base_sequence + 1u);
        seal(packet);
        break;
    case 7:
        encode(session, base_sequence + 1u, line.data(), length, packet);
        break;
    }
    return packet;
}

} // namespace bridge::mock
