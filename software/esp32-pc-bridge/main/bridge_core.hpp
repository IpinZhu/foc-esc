#pragma once

#include "bridge_protocol.hpp"

namespace bridge {

constexpr std::size_t kQueueDepth = 8;
constexpr std::uint64_t kSessionSilenceMs = 1000;

enum class ReceiveResult { accepted, source, packet, line, stale, session, full };

struct Counters {
    std::uint32_t accepted = 0;
    std::uint32_t wrong_source = 0;
    std::uint32_t invalid_packet = 0;
    std::uint32_t invalid_line = 0;
    std::uint32_t stale = 0;
    std::uint32_t session_rejected = 0;
    std::uint32_t output_full = 0;
    std::uint32_t completed = 0;
};

class Receiver {
public:
    explicit Receiver(Mac source) : source_(source) {}

    ReceiveResult receive(const Mac& source, const std::uint8_t* data,
                          std::size_t size, std::uint64_t now_ms) {
        if (source != source_) {
            ++counters_.wrong_source;
            return ReceiveResult::source;
        }
        Frame frame;
        if (!decode(data, size, frame)) {
            ++counters_.invalid_packet;
            return ReceiveResult::packet;
        }
        if (!complete_line(frame.payload, frame.length)) {
            ++counters_.invalid_line;
            return ReceiveResult::line;
        }
        if (have_sequence_) {
            if (frame.session != session_) {
                // A new boot may take over only after the old session falls silent.
                if (now_ms - last_received_ms_ < kSessionSilenceMs) {
                    ++counters_.session_rejected;
                    return ReceiveResult::session;
                }
            } else {
                const std::uint32_t distance = frame.sequence - sequence_;
                if (distance == 0 || distance >= 0x80000000u) {
                    ++counters_.stale;
                    return ReceiveResult::stale;
                }
            }
        }
        have_sequence_ = true;
        session_ = frame.session;
        sequence_ = frame.sequence;
        last_received_ms_ = now_ms;
        if (count_ == kQueueDepth) {
            ++counters_.output_full;
            return ReceiveResult::full;
        }
        auto& line = lines_[(head_ + count_) % kQueueDepth];
        std::memcpy(line.bytes.data(), frame.payload, frame.length);
        line.size = frame.length;
        line.offset = 0;
        ++count_;
        ++counters_.accepted;
        return ReceiveResult::accepted;
    }

    template <typename Sink>
    void pump(Sink& sink) {
        if (count_ == 0) {
            return;
        }
        auto& line = lines_[head_];
        const auto remaining = line.size - line.offset;
        const auto written = sink.write(line.bytes.data() + line.offset, remaining);
        line.offset += written;
        if (line.offset == line.size) {
            head_ = (head_ + 1) % kQueueDepth;
            --count_;
            ++counters_.completed;
        }
    }

    std::size_t pending() const { return count_; }
    const Counters& counters() const { return counters_; }

private:
    struct Line {
        std::array<std::uint8_t, kMaxPayload> bytes{};
        std::size_t size = 0;
        std::size_t offset = 0;
    };
    Mac source_;
    std::array<Line, kQueueDepth> lines_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
    bool have_sequence_ = false;
    std::uint32_t session_ = 0;
    std::uint32_t sequence_ = 0;
    std::uint64_t last_received_ms_ = 0;
    Counters counters_{};
};

} // namespace bridge
