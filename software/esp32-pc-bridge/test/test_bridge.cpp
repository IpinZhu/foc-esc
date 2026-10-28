#include "bridge_core.hpp"
#include "mock_foc.hpp"
#include "wire_v2.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n";        \
      std::exit(1);                                                            \
    }                                                                          \
  } while (false)

using namespace bridge;
const Mac source{0x02, 0x46, 0x4f, 0x43, 0, 1};

struct FakeUsb {
  std::string bytes;
  std::size_t capacity = 1024;
  std::size_t write(const std::uint8_t *data, std::size_t size) {
    const auto count = std::min(size, capacity);
    bytes.append(reinterpret_cast<const char *>(data), count);
    return count;
  }
};

Packet packet(std::uint32_t seq = 0, std::uint32_t session = 1,
              const std::string &line = "ok\r\n") {
  Packet out;
  CHECK(encode(session, seq,
               reinterpret_cast<const std::uint8_t *>(line.data()), line.size(),
               out));
  return out;
}

ReceiveResult deliver(Receiver &receiver, const Packet &input,
                      std::uint64_t now = 0, const Mac &mac = source) {
  return receiver.receive(mac, input.bytes.data(), input.size, now);
}

void drain(Receiver &receiver, FakeUsb &usb) {
  for (std::size_t i = 0; i < kQueueDepth * kMaxPayload && receiver.pending();
       ++i) {
    receiver.pump(usb);
  }
  CHECK(receiver.pending() == 0);
}

void crc_and_wire() {
  const std::string known = "123456789";
  CHECK(crc16(reinterpret_cast<const std::uint8_t *>(known.data()),
              known.size()) == 0x29b1);
  const auto input = packet(0x55667788, 0x11223344, "A\r\n");
  const std::vector<std::uint8_t> expected{0x46, 0x43, 1,    3,    0x44, 0x33,
                                           0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
                                           0x41, 0x0d, 0x0a, 0x5e, 0x13};
  CHECK(input.size == expected.size());
  CHECK(std::equal(expected.begin(), expected.end(), input.bytes.begin()));
  Frame frame;
  CHECK(decode(input.bytes.data(), input.size, frame));
  CHECK(frame.session == 0x11223344 && frame.sequence == 0x55667788);
  CHECK(frame.length == 3 && std::memcmp(frame.payload, "A\r\n", 3) == 0);
}

void sizes_and_corruption() {
  Frame frame;
  auto input = packet();
  for (std::size_t size = 0; size < input.size; ++size) {
    CHECK(!decode(input.bytes.data(), size, frame));
  }
  CHECK(!decode(nullptr, input.size, frame));
  CHECK(!decode(input.bytes.data(), kMaxPacket + 1, frame));
  for (std::size_t i = 0; i < input.size; ++i) {
    auto bad = input;
    bad.bytes[i] ^= 1;
    CHECK(!decode(bad.bytes.data(), bad.size, frame));
  }
  for (const std::size_t index : {0u, 1u, 2u, 3u}) {
    auto bad = input;
    ++bad.bytes[index];
    seal(bad);
    CHECK(!decode(bad.bytes.data(), bad.size, frame));
  }
  input = packet(0, 1, std::string(kMaxPayload - 2, 'x') + "\r\n");
  CHECK(input.size == kMaxPacket);
  CHECK(decode(input.bytes.data(), input.size, frame));
  Receiver receiver(source);
  CHECK(deliver(receiver, input) == ReceiveResult::accepted);
  FakeUsb usb;
  drain(receiver, usb);
  CHECK(usb.bytes == std::string(kMaxPayload - 2, 'x') + "\r\n");
  const std::string too_long(kMaxPayload + 1, 'x');
  CHECK(!encode(1, 0, reinterpret_cast<const std::uint8_t *>(too_long.data()),
                too_long.size(), input));
  CHECK(input.size == 0);
  CHECK(!encode(1, 0, nullptr, 1, input));
  CHECK(!encode(1, 0, reinterpret_cast<const std::uint8_t *>(""), 0, input));
}

void line_validation() {
  const std::vector<std::string> invalid{"\r\n",
                                         "ok",
                                         "ok\n",
                                         "ok\r",
                                         "a\nb\r\n",
                                         "a\rb\r\n",
                                         std::string("a\0b\r\n", 6),
                                         std::string("a\x80\r\n", 5),
                                         "a\tb\r\n"};
  Receiver receiver(source);
  for (const auto &line : invalid) {
    CHECK(!complete_line(reinterpret_cast<const std::uint8_t *>(line.data()),
                         line.size()));
    CHECK(deliver(receiver, packet(0, 1, line)) == ReceiveResult::line);
  }
  CHECK(receiver.pending() == 0);
  CHECK(deliver(receiver, packet(0, 1, "A\r\n")) == ReceiveResult::accepted);
}

void source_and_mac() {
  Mac mac{};
  CHECK(parse_mac("02:46:4F:43:00:01", mac) && mac == source &&
        unicast_mac(mac));
  for (const char *invalid : {"", "2:46:4f:43:00:01", "02-46:4f:43:00:01",
                              "02:46:4f:43:00:0g", "02:46:4f:43:00:001"}) {
    CHECK(!parse_mac(invalid, mac));
    CHECK(mac == source);
  }
  CHECK(!parse_mac(nullptr, mac));
  CHECK(!unicast_mac(Mac{}));
  CHECK(!unicast_mac(Mac{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}));
  Receiver receiver(source);
  CHECK(deliver(receiver, packet(), 0, Mac{}) == ReceiveResult::source);
  CHECK(deliver(receiver, packet()) == ReceiveResult::accepted);
  CHECK(receiver.counters().wrong_source == 1);
}

void sequence_and_sessions() {
  Receiver receiver(source);
  CHECK(deliver(receiver, packet(0xfffffffeu), 100) == ReceiveResult::accepted);
  CHECK(deliver(receiver, packet(0xffffffffu), 110) == ReceiveResult::accepted);
  CHECK(deliver(receiver, packet(0), 120) == ReceiveResult::accepted);
  CHECK(deliver(receiver, packet(0), 130) == ReceiveResult::stale);
  CHECK(deliver(receiver, packet(0xffffffffu), 140) == ReceiveResult::stale);
  CHECK(deliver(receiver, packet(0x80000000u), 150) == ReceiveResult::stale);
  CHECK(deliver(receiver, packet(3), 160) == ReceiveResult::accepted);
  CHECK(deliver(receiver, packet(0, 2), 500) == ReceiveResult::session);
  CHECK(deliver(receiver, packet(0, 2), 1159) == ReceiveResult::session);
  CHECK(deliver(receiver, packet(0, 2), 1160) == ReceiveResult::accepted);
  CHECK(deliver(receiver, packet(4, 1), 1161) == ReceiveResult::session);
  CHECK(deliver(receiver, packet(0, 2), 2161) == ReceiveResult::stale);
  CHECK(receiver.counters().stale == 4);
  CHECK(receiver.counters().session_rejected == 3);
}

void backpressure_and_short_writes() {
  Receiver receiver(source);
  FakeUsb usb;
  usb.capacity = 0;
  CHECK(deliver(receiver, packet(0, 1, "first\r\n")) ==
        ReceiveResult::accepted);
  for (int i = 0; i < 10; ++i)
    receiver.pump(usb);
  CHECK(receiver.pending() == 1 && usb.bytes.empty());
  usb.capacity = 2;
  receiver.pump(usb);
  CHECK(usb.bytes == "fi");
  CHECK(deliver(receiver, packet(1, 1, "second\r\n")) ==
        ReceiveResult::accepted);
  usb.capacity = 0;
  receiver.pump(usb);
  CHECK(usb.bytes == "fi");
  usb.capacity = 1;
  drain(receiver, usb);
  CHECK(usb.bytes == "first\r\nsecond\r\n");
  CHECK(receiver.counters().completed == 2);
}

void bounded_queue() {
  Receiver receiver(source);
  std::string expected;
  for (std::uint32_t i = 0; i < kQueueDepth; ++i) {
    const auto line = std::to_string(i) + "\r\n";
    CHECK(deliver(receiver, packet(i, 1, line)) == ReceiveResult::accepted);
    expected += line;
  }
  CHECK(deliver(receiver, packet(kQueueDepth)) == ReceiveResult::full);
  CHECK(receiver.pending() == kQueueDepth);
  FakeUsb usb;
  drain(receiver, usb);
  CHECK(usb.bytes == expected);
  CHECK(deliver(receiver, packet(kQueueDepth)) == ReceiveResult::stale);
  CHECK(deliver(receiver, packet(kQueueDepth + 1, 1, "recovered\r\n")) ==
        ReceiveResult::accepted);
  drain(receiver, usb);
  CHECK(usb.bytes == expected + "recovered\r\n");
  CHECK(receiver.counters().output_full == 1);
}

void mock_end_to_end() {
  Receiver receiver(source);
  FakeUsb usb;
  usb.capacity = 3;
  std::string expected;
  constexpr std::uint32_t steps = 160;
  for (std::uint32_t step = 0; step < steps; ++step) {
    const auto input = mock::scenario(42, step);
    const auto result = deliver(receiver, input, step * mock::kStepMs);
    const auto stage = step % mock::kStepsPerCycle;
    if (stage == 0 || stage == 7) {
      CHECK(result == ReceiveResult::accepted);
      std::array<std::uint8_t, kMaxPayload> line{};
      std::size_t length = 0;
      CHECK(mock::telemetry((step / 8) * 2 + (stage == 7), line, length));
      expected.append(reinterpret_cast<const char *>(line.data()), length);
    } else if (stage == 1 || stage == 4) {
      CHECK(result == ReceiveResult::stale);
    } else if (stage == 6) {
      CHECK(result == ReceiveResult::line);
    } else {
      CHECK(result == ReceiveResult::packet);
    }
    drain(receiver, usb);
  }
  CHECK(usb.bytes == expected);
  CHECK(receiver.counters().accepted == 40);
  CHECK(receiver.counters().completed == 40);
  CHECK(receiver.counters().invalid_packet == 60);
  CHECK(receiver.counters().invalid_line == 20);
  CHECK(receiver.counters().stale == 40);
}

void randomized_decoder_bounds() {
  std::array<std::uint8_t, 300> bytes{};
  std::uint32_t rng = 12345;
  Frame frame;
  for (unsigned iteration = 0; iteration < 10000; ++iteration) {
    for (auto &byte : bytes) {
      rng = rng * 1664525u + 1013904223u;
      byte = static_cast<std::uint8_t>(rng >> 24);
    }
    const auto size = iteration % (bytes.size() + 1);
    if (decode(bytes.data(), size, frame)) {
      CHECK(frame.length <= kMaxPayload);
      CHECK(frame.payload + frame.length <= bytes.data() + size - 2);
    }
  }
}

void v2_vector_stream_and_bounds() {
  using bridge::v2::Decoder;
  using bridge::v2::Frame;
  using bridge::v2::Packet;
  using namespace bridge::v2;
  const std::uint8_t payload[] = {'s', 't', 'a', 't', 'u', 's'};
  Frame input{request,
              0x0102030405060708ull,
              0x1112131415161718ull,
              0x21222324,
              0x31323334,
              0x41424344,
              payload,
              6};
  Packet p;
  CHECK(encode(input, p));
  CHECK(p.size == 42);
  const std::uint8_t expected[] = {
      0x46, 0x43, 0x02, 0x03, 0x06, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04,
      0x03, 0x02, 0x01, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,
      0x24, 0x23, 0x22, 0x21, 0x34, 0x33, 0x32, 0x31, 0x44, 0x43, 0x42,
      0x41, 0x73, 0x74, 0x61, 0x74, 0x75, 0x73, 0x12, 0x1d};
  CHECK(std::equal(std::begin(expected), std::end(expected), p.bytes.begin()));
  Frame out;
  CHECK(decode(p.bytes.data(), p.size, out));
  CHECK(out.kind == request && out.session == input.session &&
        out.boot == input.boot && out.request == input.request &&
        out.sequence == input.sequence && out.permit == input.permit &&
        out.length == 6);
  auto damaged = p;
  damaged.bytes[37] ^= 1;
  CHECK(!decode(damaged.bytes.data(), damaged.size, out));
  Decoder d;
  Packet empty;
  Frame h{hello, 7, 8, 9, 10, 11, nullptr, 0};
  CHECK(encode(h, empty));
  for (std::size_t i = 0; i < empty.size; i++)
    CHECK(d.push(empty.bytes[i], out) == (i + 1 == empty.size));
  Packet combo;
  std::memcpy(combo.bytes.data(), p.bytes.data(), p.size);
  std::memcpy(combo.bytes.data() + p.size, empty.bytes.data(), empty.size);
  d.reset();
  bool got = false;
  for (std::size_t i = 0; i < p.size + empty.size; i++)
    got = d.push(combo.bytes[i], out) || got;
  CHECK(got);
  Frame too_large{request, 1, 2, 3, 4, 5, payload, bridge::v2::kMaxPayload + 1};
  CHECK(!encode(too_large, empty));
}

int main() {
  v2_vector_stream_and_bounds();
  crc_and_wire();
  sizes_and_corruption();
  line_validation();
  source_and_mac();
  sequence_and_sessions();
  backpressure_and_short_writes();
  bounded_queue();
  mock_end_to_end();
  randomized_decoder_bounds();
  std::cout << "PASS: 9 C++ interface test groups (including 10,000 malformed "
               "packets)\n";
}
