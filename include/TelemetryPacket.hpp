#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace telemetry {

/// Sync word used to delimit packet frames on the wire.
constexpr uint32_t kSyncWord = 0xDEADBEEFu;

/// Maximum payload size for a single telemetry frame (bytes).
constexpr std::size_t kMaxPayloadSize = 256;

/// Status flag bit definitions (bitwise OR combinable).
enum StatusFlag : uint8_t {
    FLAG_NONE           = 0x00,
    FLAG_SENSOR_OK      = 0x01,
    FLAG_SENSOR_WARN    = 0x02,
    FLAG_SENSOR_FAULT   = 0x04,
    FLAG_COMM_ERROR     = 0x08,
    FLAG_HARDWARE_FAULT = 0x10,
    FLAG_OVERTEMP       = 0x20,
    FLAG_LOW_POWER      = 0x40,
    FLAG_RECOVERY       = 0x80,
};

#pragma pack(push, 1)
/// Packed binary telemetry header (9 bytes).
struct TelemetryHeader {
    uint32_t sync{kSyncWord};   ///< Frame sync word (0xDEADBEEF)
    uint16_t sequence_id{0};    ///< Monotonic sequence counter
    uint16_t payload_length{0}; ///< Payload length in bytes
    uint8_t  status_flags{FLAG_NONE}; ///< Bitwise sensor/error status
};
#pragma pack(pop)

static_assert(sizeof(TelemetryHeader) == 9, "TelemetryHeader must be tightly packed");

/// Complete telemetry packet: header + payload + CRC-16 trailer.
struct TelemetryPacket {
    TelemetryHeader header{};
    std::array<uint8_t, kMaxPayloadSize> payload{};
    uint16_t crc16{0};
};

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — common in embedded links
// ---------------------------------------------------------------------------

inline uint16_t crc16_ccitt(const uint8_t* data, std::size_t length) {
    uint16_t crc = 0xFFFFu;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000u) != 0) {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
            } else {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

inline uint16_t compute_packet_crc(const TelemetryPacket& packet) {
    std::array<uint8_t, sizeof(TelemetryHeader) + kMaxPayloadSize> buffer{};
    std::memcpy(buffer.data(), &packet.header, sizeof(TelemetryHeader));
    const auto len = static_cast<std::size_t>(packet.header.payload_length);
    if (len > kMaxPayloadSize) {
        throw std::out_of_range("payload_length exceeds kMaxPayloadSize");
    }
    if (len > 0) {
        std::memcpy(buffer.data() + sizeof(TelemetryHeader), packet.payload.data(), len);
    }
    return crc16_ccitt(buffer.data(), sizeof(TelemetryHeader) + len);
}

// ---------------------------------------------------------------------------
// Bitwise status-flag helpers
// ---------------------------------------------------------------------------

inline void set_flag(uint8_t& flags, StatusFlag flag) {
    flags = static_cast<uint8_t>(flags | static_cast<uint8_t>(flag));
}

inline void clear_flag(uint8_t& flags, StatusFlag flag) {
    flags = static_cast<uint8_t>(flags & static_cast<uint8_t>(~static_cast<uint8_t>(flag)));
}

inline bool has_flag(uint8_t flags, StatusFlag flag) {
    return (flags & static_cast<uint8_t>(flag)) != 0;
}

inline void set_flag(TelemetryPacket& packet, StatusFlag flag) {
    set_flag(packet.header.status_flags, flag);
}

inline void clear_flag(TelemetryPacket& packet, StatusFlag flag) {
    clear_flag(packet.header.status_flags, flag);
}

inline bool has_flag(const TelemetryPacket& packet, StatusFlag flag) {
    return has_flag(packet.header.status_flags, flag);
}

inline std::string flags_to_string(uint8_t flags) {
    if (flags == FLAG_NONE) {
        return "NONE";
    }
    std::string result;
    auto append = [&](StatusFlag f, const char* name) {
        if (has_flag(flags, f)) {
            if (!result.empty()) {
                result += '|';
            }
            result += name;
        }
    };
    append(FLAG_SENSOR_OK, "SENSOR_OK");
    append(FLAG_SENSOR_WARN, "SENSOR_WARN");
    append(FLAG_SENSOR_FAULT, "SENSOR_FAULT");
    append(FLAG_COMM_ERROR, "COMM_ERROR");
    append(FLAG_HARDWARE_FAULT, "HARDWARE_FAULT");
    append(FLAG_OVERTEMP, "OVERTEMP");
    append(FLAG_LOW_POWER, "LOW_POWER");
    append(FLAG_RECOVERY, "RECOVERY");
    return result;
}

// ---------------------------------------------------------------------------
// Pack / unpack
// ---------------------------------------------------------------------------

/// Serialize a TelemetryPacket into a contiguous byte buffer (header|payload|crc).
inline std::vector<uint8_t> pack_packet(const TelemetryPacket& packet) {
    const auto payload_len = static_cast<std::size_t>(packet.header.payload_length);
    if (payload_len > kMaxPayloadSize) {
        throw std::out_of_range("payload_length exceeds kMaxPayloadSize");
    }

    TelemetryPacket working = packet;
    working.header.sync = kSyncWord;
    working.crc16 = compute_packet_crc(working);

    std::vector<uint8_t> out(sizeof(TelemetryHeader) + payload_len + sizeof(uint16_t));
    std::memcpy(out.data(), &working.header, sizeof(TelemetryHeader));
    if (payload_len > 0) {
        std::memcpy(out.data() + sizeof(TelemetryHeader), working.payload.data(), payload_len);
    }
    std::memcpy(out.data() + sizeof(TelemetryHeader) + payload_len, &working.crc16, sizeof(uint16_t));
    return out;
}

/// Deserialize a byte buffer into a TelemetryPacket. Does not validate CRC.
inline TelemetryPacket unpack_packet(const uint8_t* data, std::size_t size) {
    constexpr std::size_t kMinSize = sizeof(TelemetryHeader) + sizeof(uint16_t);
    if (data == nullptr || size < kMinSize) {
        throw std::invalid_argument("buffer too small for TelemetryPacket");
    }

    TelemetryPacket packet{};
    std::memcpy(&packet.header, data, sizeof(TelemetryHeader));

    if (packet.header.sync != kSyncWord) {
        throw std::invalid_argument("invalid sync word");
    }

    const auto payload_len = static_cast<std::size_t>(packet.header.payload_length);
    if (payload_len > kMaxPayloadSize) {
        throw std::out_of_range("payload_length exceeds kMaxPayloadSize");
    }
    if (size < sizeof(TelemetryHeader) + payload_len + sizeof(uint16_t)) {
        throw std::invalid_argument("buffer truncated relative to payload_length");
    }

    if (payload_len > 0) {
        std::memcpy(packet.payload.data(), data + sizeof(TelemetryHeader), payload_len);
    }
    std::memcpy(&packet.crc16, data + sizeof(TelemetryHeader) + payload_len, sizeof(uint16_t));
    return packet;
}

inline TelemetryPacket unpack_packet(const std::vector<uint8_t>& buffer) {
    return unpack_packet(buffer.data(), buffer.size());
}

/// Validate CRC-16 over header + payload against the stored trailer.
inline bool validate_crc(const TelemetryPacket& packet) {
    try {
        return packet.crc16 == compute_packet_crc(packet);
    } catch (const std::out_of_range&) {
        return false;
    }
}

/// Build a fully formed packet (sync, length, CRC computed).
inline TelemetryPacket make_packet(uint16_t sequence_id,
                                   uint8_t status_flags,
                                   const uint8_t* payload_data,
                                   uint16_t payload_length) {
    if (payload_length > kMaxPayloadSize) {
        throw std::out_of_range("payload_length exceeds kMaxPayloadSize");
    }

    TelemetryPacket packet{};
    packet.header.sync = kSyncWord;
    packet.header.sequence_id = sequence_id;
    packet.header.payload_length = payload_length;
    packet.header.status_flags = status_flags;

    if (payload_data != nullptr && payload_length > 0) {
        std::memcpy(packet.payload.data(), payload_data, payload_length);
    }

    packet.crc16 = compute_packet_crc(packet);
    return packet;
}

}  // namespace telemetry
