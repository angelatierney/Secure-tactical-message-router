#include "EmbeddedFSM.hpp"

#include <sstream>

namespace fsm {
namespace {

constexpr uint32_t kErrorThresholdDegraded = 3;
constexpr uint32_t kErrorThresholdFault    = 8;
constexpr uint32_t kGoodThresholdRecover   = 5;

using clock = std::chrono::high_resolution_clock;

bool is_critical_fault(uint8_t flags) {
    using namespace telemetry;
    return has_flag(flags, FLAG_HARDWARE_FAULT) ||
           has_flag(flags, FLAG_SENSOR_FAULT) ||
           (has_flag(flags, FLAG_OVERTEMP) && has_flag(flags, FLAG_COMM_ERROR));
}

bool is_degraded_condition(uint8_t flags) {
    using namespace telemetry;
    return has_flag(flags, FLAG_SENSOR_WARN) ||
           has_flag(flags, FLAG_COMM_ERROR) ||
           has_flag(flags, FLAG_LOW_POWER) ||
           has_flag(flags, FLAG_OVERTEMP);
}

bool is_healthy(uint8_t flags) {
    using namespace telemetry;
    return has_flag(flags, FLAG_SENSOR_OK) &&
           !is_critical_fault(flags) &&
           !is_degraded_condition(flags);
}

}  // namespace

const char* to_string(NodeState state) noexcept {
    switch (state) {
        case NodeState::INIT:        return "INIT";
        case NodeState::OPERATIONAL: return "OPERATIONAL";
        case NodeState::DEGRADED:    return "DEGRADED";
        case NodeState::FAULT:       return "FAULT";
    }
    return "UNKNOWN";
}

EmbeddedFSM::EmbeddedFSM(Logger& logger) : logger_(logger) {
    logger_.log(LogLevel::Info, "EmbeddedFSM constructed; state=INIT");
}

void EmbeddedFSM::reset() {
    state_ = NodeState::INIT;
    last_sequence_.reset();
    consecutive_errors_ = 0;
    consecutive_goods_ = 0;
    logger_.log(LogLevel::Warn, "EmbeddedFSM reset to INIT");
}

std::string EmbeddedFSM::trigger_recovery(NodeState target) {
    ++recovery_count_;

    std::ostringstream oss;
    switch (target) {
        case NodeState::DEGRADED:
            oss << "RECOVERY[DEGRADED]: soft-reset sensors, widen CRC window, "
                << "throttle TX duty-cycle (event #" << recovery_count_ << ")";
            logger_.log(LogLevel::Warn, oss.str());
            break;
        case NodeState::FAULT:
            oss << "RECOVERY[FAULT]: isolate RF front-end, dump diagnostic "
                << "registers, schedule cold restart (event #" << recovery_count_ << ")";
            logger_.log(LogLevel::Error, oss.str());
            break;
        default:
            oss << "RECOVERY: no-op for target state " << to_string(target);
            break;
    }
    return oss.str();
}

NodeState EmbeddedFSM::evaluate_transition(const telemetry::TelemetryPacket& packet,
                                           bool crc_valid,
                                           bool sequence_ok,
                                           bool deadline_met) {
    const uint8_t flags = packet.header.status_flags;
    const bool recovery_flag = telemetry::has_flag(flags, telemetry::FLAG_RECOVERY);

    if (!crc_valid || !deadline_met) {
        ++consecutive_errors_;
        consecutive_goods_ = 0;
    } else if (is_critical_fault(flags)) {
        consecutive_errors_ = kErrorThresholdFault;
        consecutive_goods_ = 0;
    } else if (is_degraded_condition(flags) || !sequence_ok) {
        ++consecutive_errors_;
        consecutive_goods_ = 0;
    } else if (is_healthy(flags) || recovery_flag) {
        ++consecutive_goods_;
        if (consecutive_goods_ >= kGoodThresholdRecover) {
            consecutive_errors_ = 0;
        }
    }

    NodeState next = state_;

    switch (state_) {
        case NodeState::INIT:
            if (crc_valid && sequence_ok && deadline_met &&
                (is_healthy(flags) || recovery_flag)) {
                next = NodeState::OPERATIONAL;
            } else if (is_critical_fault(flags) || consecutive_errors_ >= kErrorThresholdFault) {
                next = NodeState::FAULT;
            } else if (consecutive_errors_ >= kErrorThresholdDegraded) {
                next = NodeState::DEGRADED;
            }
            break;

        case NodeState::OPERATIONAL:
            if (is_critical_fault(flags) || consecutive_errors_ >= kErrorThresholdFault) {
                next = NodeState::FAULT;
            } else if (consecutive_errors_ >= kErrorThresholdDegraded ||
                       is_degraded_condition(flags)) {
                next = NodeState::DEGRADED;
            }
            break;

        case NodeState::DEGRADED:
            if (is_critical_fault(flags) || consecutive_errors_ >= kErrorThresholdFault) {
                next = NodeState::FAULT;
            } else if (consecutive_goods_ >= kGoodThresholdRecover && is_healthy(flags)) {
                next = NodeState::OPERATIONAL;
            }
            break;

        case NodeState::FAULT:
            // Explicit recovery flag + sustained good packets required to leave FAULT.
            if (recovery_flag && consecutive_goods_ >= kGoodThresholdRecover &&
                crc_valid && is_healthy(flags)) {
                next = NodeState::DEGRADED;
            }
            break;
    }

    return next;
}

ProcessResult EmbeddedFSM::processPacket(const telemetry::TelemetryPacket& packet) {
    const auto t0 = clock::now();

    ProcessResult result{};
    result.previous_state = state_;

    result.crc_valid = telemetry::validate_crc(packet);
    if (!result.crc_valid) {
        ++crc_failures_;
    }

    if (!last_sequence_.has_value()) {
        result.sequence_ok = true;
    } else {
        const uint16_t expected =
            static_cast<uint16_t>(last_sequence_.value() + 1u);
        result.sequence_ok = (packet.header.sequence_id == expected);
    }
    last_sequence_ = packet.header.sequence_id;

    // Measure integrity checks + transition evaluation against the 5 ms deadline.
    // Deadline is applied to the transition decision in a single pass (no double-count).
    const auto t_pre_eval = clock::now();
    const auto pre_elapsed =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t_pre_eval - t0);
    // Conservative: if checks alone already blew the budget, mark miss before eval.
    const bool provisional_deadline = (pre_elapsed < kMaxProcessingLatency);

    NodeState next = evaluate_transition(packet, result.crc_valid, result.sequence_ok,
                                         provisional_deadline);

    const auto t1 = clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0);
    result.latency_us = static_cast<double>(elapsed.count()) / 1000.0;
    result.deadline_met = (elapsed < kMaxProcessingLatency);

    if (!result.deadline_met) {
        ++deadline_misses_;
        logger_.log(LogLevel::Error,
                    "Deadline miss: processing exceeded 5 ms (" +
                        std::to_string(result.latency_us) + " us)");
        // Escalate without re-running counters: deadline miss is a hard fault signal.
        if (next == NodeState::OPERATIONAL || next == NodeState::INIT) {
            next = NodeState::DEGRADED;
        } else if (next == NodeState::DEGRADED) {
            next = NodeState::FAULT;
        }
    }

    result.current_state = next;
    result.state_changed = (next != state_);

    if (result.state_changed) {
        if (next == NodeState::DEGRADED || next == NodeState::FAULT) {
            result.recovery_action = trigger_recovery(next);
        }

        std::ostringstream msg;
        msg << "STATE " << to_string(state_) << " -> " << to_string(next)
            << " seq=" << packet.header.sequence_id
            << " flags=" << telemetry::flags_to_string(packet.header.status_flags)
            << " crc=" << (result.crc_valid ? "OK" : "FAIL")
            << " seq_ok=" << (result.sequence_ok ? "Y" : "N")
            << " latency_us=" << result.latency_us;

        logger_.log_transition(static_cast<uint8_t>(state_),
                               static_cast<uint8_t>(next),
                               msg.str(),
                               result.latency_us);
        state_ = next;
    } else {
        logger_.log_metric("packet processed seq=" +
                               std::to_string(packet.header.sequence_id) +
                               " state=" + to_string(state_),
                           result.latency_us);
    }

    ++packets_processed_;
    return result;
}

}  // namespace fsm
