#include "Logger.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace fsm {
namespace {

const char* level_to_string(LogLevel level) noexcept {
    switch (level) {
        case LogLevel::Info:   return "INFO";
        case LogLevel::Warn:   return "WARN";
        case LogLevel::Error:  return "ERROR";
        case LogLevel::Metric: return "METRIC";
    }
    return "UNKNOWN";
}

std::string format_timestamp(std::chrono::system_clock::time_point tp) {
    const auto time = std::chrono::system_clock::to_time_t(tp);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp.time_since_epoch()) %
                    1000;

    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &time);
#else
    localtime_r(&time, &tm_buf);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%H:%M:%S") << '.'
        << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

}  // namespace

Logger::Logger(std::size_t capacity)
    : buffer_(capacity), capacity_(capacity == 0 ? 1 : capacity) {
    if (capacity == 0) {
        buffer_.resize(1);
    }
}

void Logger::push_unlocked(LogEntry entry) {
    buffer_[head_] = std::move(entry);
    head_ = (head_ + 1) % capacity_;
    if (count_ < capacity_) {
        ++count_;
    } else {
        ++drops_;
    }
}

void Logger::log(LogLevel level, const std::string& message) {
    LogEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.level = level;
    entry.message = message;

    std::lock_guard<std::mutex> lock(mutex_);
    push_unlocked(std::move(entry));
}

void Logger::log_transition(uint8_t from_state,
                            uint8_t to_state,
                            const std::string& message,
                            double latency_us) {
    LogEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.level = LogLevel::Warn;
    entry.message = message;
    entry.latency_us = latency_us;
    entry.from_state = from_state;
    entry.to_state = to_state;
    entry.is_transition = true;

    std::lock_guard<std::mutex> lock(mutex_);
    push_unlocked(std::move(entry));
}

void Logger::log_metric(const std::string& message, double latency_us) {
    LogEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.level = LogLevel::Metric;
    entry.message = message;
    entry.latency_us = latency_us;

    std::lock_guard<std::mutex> lock(mutex_);
    push_unlocked(std::move(entry));
}

std::size_t Logger::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
}

std::size_t Logger::drop_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return drops_;
}

std::vector<LogEntry> Logger::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<LogEntry> out;
    out.reserve(count_);

    if (count_ == 0) {
        return out;
    }

    const std::size_t start =
        (count_ < capacity_) ? 0 : head_;  // oldest slot when full

    for (std::size_t i = 0; i < count_; ++i) {
        const std::size_t idx = (start + i) % capacity_;
        out.push_back(buffer_[idx]);
    }
    return out;
}

void Logger::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    head_ = 0;
    count_ = 0;
    drops_ = 0;
}

void Logger::print_summary() const {
    const auto entries = snapshot();

    std::size_t transitions = 0;
    std::size_t metrics = 0;
    double min_us = 0.0;
    double max_us = 0.0;
    double sum_us = 0.0;
    bool have_latency = false;

    std::cout << "\n========== Telemetry / FSM Log Summary ==========\n";

    for (const auto& e : entries) {
        if (e.is_transition) {
            ++transitions;
            std::cout << '[' << format_timestamp(e.timestamp) << "] "
                      << level_to_string(e.level) << "  " << e.message << '\n';
        }
        if (e.level == LogLevel::Metric || e.is_transition) {
            if (e.latency_us > 0.0 || e.level == LogLevel::Metric) {
                ++metrics;
                sum_us += e.latency_us;
                if (!have_latency) {
                    min_us = max_us = e.latency_us;
                    have_latency = true;
                } else {
                    min_us = std::min(min_us, e.latency_us);
                    max_us = std::max(max_us, e.latency_us);
                }
            }
        }
        if (e.level == LogLevel::Error && !e.is_transition) {
            std::cout << '[' << format_timestamp(e.timestamp) << "] "
                      << level_to_string(e.level) << "  " << e.message << '\n';
        }
    }

    std::cout << "-------------------------------------------------\n";
    std::cout << "Ring capacity : " << capacity_ << '\n';
    std::cout << "Entries stored: " << entries.size() << '\n';
    std::cout << "Dropped       : " << drop_count() << '\n';
    std::cout << "Transitions   : " << transitions << '\n';
    if (have_latency && metrics > 0) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Latency min   : " << min_us << " us\n";
        std::cout << "Latency max   : " << max_us << " us\n";
        std::cout << "Latency avg   : " << (sum_us / static_cast<double>(metrics))
                  << " us\n";
    }
    std::cout << "=================================================\n";
}

}  // namespace fsm
