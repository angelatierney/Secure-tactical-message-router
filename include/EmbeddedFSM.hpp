#pragma once

#include "Logger.hpp"
#include "TelemetryPacket.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace fsm {

/// Deterministic hard deadline for a single packet evaluation cycle.
constexpr auto kMaxProcessingLatency = std::chrono::milliseconds(5);

enum class NodeState : uint8_t {
    INIT = 0,
    OPERATIONAL,
    DEGRADED,
    FAULT,
};

[[nodiscard]] const char* to_string(NodeState state) noexcept;

struct ProcessResult {
    NodeState previous_state{NodeState::INIT};
    NodeState current_state{NodeState::INIT};
    bool      state_changed{false};
    bool      crc_valid{false};
    bool      sequence_ok{false};
    bool      deadline_met{false};
    double    latency_us{0.0};
    std::string recovery_action;
};

/// Finite-state controller for an embedded tactical radio / sensor node.
class EmbeddedFSM {
public:
    explicit EmbeddedFSM(Logger& logger);

    EmbeddedFSM(const EmbeddedFSM&) = delete;
    EmbeddedFSM& operator=(const EmbeddedFSM&) = delete;

    /// Evaluate packet integrity and status flags; drive state transitions.
    /// Timing is measured with high_resolution_clock; deadline is < 5 ms.
    ProcessResult processPacket(const telemetry::TelemetryPacket& packet);

    [[nodiscard]] NodeState state() const noexcept { return state_; }
    [[nodiscard]] std::optional<uint16_t> last_sequence() const noexcept {
        return last_sequence_;
    }
    [[nodiscard]] uint64_t  packets_processed() const noexcept { return packets_processed_; }
    [[nodiscard]] uint64_t  crc_failures() const noexcept { return crc_failures_; }
    [[nodiscard]] uint64_t  deadline_misses() const noexcept { return deadline_misses_; }
    [[nodiscard]] uint64_t  recovery_count() const noexcept { return recovery_count_; }

    /// Force a clean restart into INIT (e.g. simulated power-on reset).
    void reset();

private:
    NodeState evaluate_transition(const telemetry::TelemetryPacket& packet,
                                  bool crc_valid,
                                  bool sequence_ok,
                                  bool deadline_met);

    std::string trigger_recovery(NodeState target);

    Logger&   logger_;
    NodeState state_{NodeState::INIT};
    std::optional<uint16_t> last_sequence_;
    uint32_t  consecutive_errors_{0};
    uint32_t  consecutive_goods_{0};
    uint64_t  packets_processed_{0};
    uint64_t  crc_failures_{0};
    uint64_t  deadline_misses_{0};
    uint64_t  recovery_count_{0};
};

}  // namespace fsm
