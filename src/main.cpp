#include "EmbeddedFSM.hpp"
#include "Logger.hpp"
#include "TelemetryPacket.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

namespace {

constexpr int kLoopHz = 100;
constexpr int kTotalTicks = 200;  // 2.0 seconds of simulated wall time
constexpr auto kTickPeriod = std::chrono::microseconds(1'000'000 / kLoopHz);

struct SimStats {
    uint64_t valid{0};
    uint64_t ooo{0};
    uint64_t fault_flagged{0};
    uint64_t crc_corrupt{0};
    uint64_t degraded_flagged{0};
};

telemetry::TelemetryPacket build_sensor_payload(uint16_t seq,
                                                uint8_t flags,
                                                std::mt19937& rng) {
    std::array<uint8_t, 16> sensor{};
    std::uniform_int_distribution<int> dist(0, 255);
    for (auto& b : sensor) {
        b = static_cast<uint8_t>(dist(rng));
    }
    // Embed a simple temperature / RSSI pattern for realism.
    sensor[0] = static_cast<uint8_t>(40 + (seq % 20));  // temperature proxy
    sensor[1] = static_cast<uint8_t>(180 - (seq % 40)); // RSSI proxy

    return telemetry::make_packet(seq, flags, sensor.data(),
                                  static_cast<uint16_t>(sensor.size()));
}

/// Scenario script: mix of healthy, out-of-order, degraded, and fault traffic.
telemetry::TelemetryPacket generate_packet(int tick,
                                           uint16_t& next_seq,
                                           SimStats& stats,
                                           std::mt19937& rng) {
    using namespace telemetry;

    // Scripted anomalies at known ticks for deterministic demo output.
    if (tick == 25) {
        // Out-of-order: skip ahead then the "missing" id arrives late later.
        next_seq = static_cast<uint16_t>(next_seq + 3);
        auto pkt = build_sensor_payload(next_seq, FLAG_SENSOR_OK, rng);
        ++next_seq;
        ++stats.ooo;
        ++stats.valid;
        return pkt;
    }

    if (tick == 40 || tick == 41 || tick == 42) {
        auto pkt = build_sensor_payload(next_seq++, FLAG_SENSOR_WARN | FLAG_LOW_POWER, rng);
        ++stats.degraded_flagged;
        return pkt;
    }

    if (tick == 70 || tick == 71) {
        auto pkt = build_sensor_payload(next_seq++, FLAG_COMM_ERROR | FLAG_SENSOR_WARN, rng);
        ++stats.degraded_flagged;
        return pkt;
    }

    if (tick >= 100 && tick <= 108) {
        auto pkt = build_sensor_payload(
            next_seq++, FLAG_HARDWARE_FAULT | FLAG_SENSOR_FAULT, rng);
        ++stats.fault_flagged;
        return pkt;
    }

    if (tick == 130) {
        // Corrupted CRC: flip a payload byte after CRC was computed.
        auto pkt = build_sensor_payload(next_seq++, FLAG_SENSOR_OK, rng);
        pkt.payload[0] ^= 0xFFu;
        ++stats.crc_corrupt;
        return pkt;
    }

    if (tick >= 160 && tick <= 175) {
        // Explicit recovery stream after fault.
        auto pkt = build_sensor_payload(next_seq++, FLAG_SENSOR_OK | FLAG_RECOVERY, rng);
        ++stats.valid;
        return pkt;
    }

    // Default healthy traffic (occasional mild warn noise).
    std::uniform_int_distribution<int> noise(0, 99);
    uint8_t flags = FLAG_SENSOR_OK;
    if (noise(rng) < 5) {
        flags = static_cast<uint8_t>(FLAG_SENSOR_OK | FLAG_SENSOR_WARN);
        ++stats.degraded_flagged;
    } else {
        ++stats.valid;
    }
    return build_sensor_payload(next_seq++, flags, rng);
}

void print_banner() {
    std::cout << "========================================================\n"
              << " realtime-embedded-fsm  |  100 Hz tactical node sim\n"
              << " C++17 FSM + CRC-16 telemetry + ring-buffer logger\n"
              << "========================================================\n";
}

}  // namespace

int main() {
    print_banner();

    fsm::Logger logger(/*capacity=*/2048);
    fsm::EmbeddedFSM controller(logger);

    std::mt19937 rng{0xC0FFEEu};
    uint16_t next_seq = 1;
    SimStats stats{};

    std::vector<double> latencies;
    latencies.reserve(static_cast<std::size_t>(kTotalTicks));

    const auto loop_start = std::chrono::steady_clock::now();

    for (int tick = 0; tick < kTotalTicks; ++tick) {
        const auto tick_deadline = loop_start + (tick + 1) * kTickPeriod;

        auto packet = generate_packet(tick, next_seq, stats, rng);

        // Round-trip pack/unpack to exercise the binary framing path.
        const auto wire = telemetry::pack_packet(packet);
        auto framed = telemetry::unpack_packet(wire);
        // Preserve intentional CRC corruption from the generator.
        if (tick == 130) {
            framed.payload[0] = packet.payload[0];
            framed.crc16 = packet.crc16;
        }

        const auto result = controller.processPacket(framed);
        latencies.push_back(result.latency_us);

        if (result.state_changed) {
            std::cout << std::fixed << std::setprecision(2)
                      << "[tick " << std::setw(3) << tick << "] "
                      << fsm::to_string(result.previous_state) << " -> "
                      << fsm::to_string(result.current_state)
                      << "  latency=" << result.latency_us << " us"
                      << "  flags="
                      << telemetry::flags_to_string(framed.header.status_flags);
            if (!result.recovery_action.empty()) {
                std::cout << "\n           " << result.recovery_action;
            }
            std::cout << '\n';
        }

        // Pace the loop at 100 Hz (best-effort on a general-purpose OS).
        std::this_thread::sleep_until(tick_deadline);
    }

    const auto loop_end = std::chrono::steady_clock::now();
    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             loop_end - loop_start)
                             .count();

    double min_us = latencies.empty() ? 0.0 : latencies.front();
    double max_us = min_us;
    double sum_us = 0.0;
    for (double v : latencies) {
        min_us = std::min(min_us, v);
        max_us = std::max(max_us, v);
        sum_us += v;
    }
    const double avg_us =
        latencies.empty() ? 0.0 : sum_us / static_cast<double>(latencies.size());

    std::cout << "\n---------- Simulation complete ----------\n";
    std::cout << "Ticks            : " << kTotalTicks << " @ " << kLoopHz << " Hz\n";
    std::cout << "Wall time        : " << wall_ms << " ms\n";
    std::cout << "Final state      : " << fsm::to_string(controller.state()) << '\n';
    std::cout << "Packets processed: " << controller.packets_processed() << '\n';
    std::cout << "CRC failures     : " << controller.crc_failures() << '\n';
    std::cout << "Deadline misses  : " << controller.deadline_misses() << '\n';
    std::cout << "Recovery events  : " << controller.recovery_count() << '\n';
    std::cout << "Scenario valid   : " << stats.valid
              << "  ooo=" << stats.ooo
              << "  degraded=" << stats.degraded_flagged
              << "  fault=" << stats.fault_flagged
              << "  crc_corrupt=" << stats.crc_corrupt << '\n';
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Proc latency us  : min=" << min_us
              << "  avg=" << avg_us
              << "  max=" << max_us << '\n';

    logger.print_summary();
    return 0;
}
