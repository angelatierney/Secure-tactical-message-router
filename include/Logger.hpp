#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace fsm {

enum class LogLevel : uint8_t {
    Info = 0,
    Warn,
    Error,
    Metric,
};

struct LogEntry {
    std::chrono::system_clock::time_point timestamp{};
    LogLevel    level{LogLevel::Info};
    std::string message;
    double      latency_us{0.0};
    uint8_t     from_state{0};
    uint8_t     to_state{0};
    bool        is_transition{false};
};

/// Thread-safe fixed-capacity ring buffer for FSM / telemetry event logging.
class Logger {
public:
    explicit Logger(std::size_t capacity = 1024);

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log(LogLevel level, const std::string& message);
    void log_transition(uint8_t from_state,
                        uint8_t to_state,
                        const std::string& message,
                        double latency_us);
    void log_metric(const std::string& message, double latency_us);

    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] std::size_t capacity() const noexcept { return capacity_; }
    [[nodiscard]] std::size_t drop_count() const;

    /// Snapshot of the ring contents in chronological order (oldest → newest).
    [[nodiscard]] std::vector<LogEntry> snapshot() const;

    /// Dump a human-readable summary of transitions and latency stats to stdout.
    void print_summary() const;

    void clear();

private:
    void push_unlocked(LogEntry entry);

    mutable std::mutex   mutex_;
    std::vector<LogEntry> buffer_;
    std::size_t capacity_;
    std::size_t head_{0};   ///< Next write index
    std::size_t count_{0};  ///< Occupied slots
    std::size_t drops_{0};
};

}  // namespace fsm
