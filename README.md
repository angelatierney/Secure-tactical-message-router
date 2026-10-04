# realtime-embedded-fsm

Modern C++17 real-time finite-state machine for an embedded tactical radio / sensor node. The project simulates a 100 Hz hardware loop that ingests packed telemetry frames, validates CRC-16 integrity, enforces a deterministic sub-5ms processing deadline, and drives state transitions with thread-safe ring-buffer logging.

## States

| State | Meaning |
|-------|---------|
| `INIT` | Power-on / awaiting first healthy packet |
| `OPERATIONAL` | Nominal sensor + link health |
| `DEGRADED` | Warnings, out-of-order sequence, or soft faults — recovery protocol active |
| `FAULT` | Hard hardware / sensor fault — isolated until explicit recovery stream |

## Layout

```
include/
  TelemetryPacket.hpp   Packed header, flags, CRC-16, pack/unpack
  EmbeddedFSM.hpp       FSM controller API
  Logger.hpp            Thread-safe ring-buffer logger
src/
  EmbeddedFSM.cpp
  Logger.cpp
  main.cpp              100 Hz simulation harness
CMakeLists.txt
README.md
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

```bash
./build/fsm_runner
```

The simulator emits:

- Valid sensor packets
- Out-of-order sequence IDs
- Degraded / hardware-fault status flags
- A CRC-corrupted frame
- An explicit recovery stream (`FLAG_RECOVERY`)

Stdout shows live state transitions, per-packet latency (µs), recovery actions, and a final logger summary (min/avg/max latency, transition count).

## Telemetry frame

Packed `TelemetryHeader` (9 bytes) + payload + CRC-16/CCITT trailer:

| Field | Type | Notes |
|-------|------|-------|
| Sync | `uint32_t` | `0xDEADBEEF` |
| Sequence ID | `uint16_t` | Monotonic |
| Payload length | `uint16_t` | ≤ 256 |
| Status flags | `uint8_t` | Bitwise sensor / error bits |
| Payload | `uint8_t[]` | Raw sensor bytes |
| CRC-16 | `uint16_t` | Over header + payload |

## Requirements

- CMake ≥ 3.16
- C++17 compiler (GCC, Clang, or MSVC)
- POSIX threads (or Windows threads via the C++ standard library)
