# openbsw-rust

Incremental Rust rewrite of [Eclipse OpenBSW](https://github.com/eclipse-openbsw/openbsw) — an open-source AUTOSAR-inspired BSW stack for automotive microcontrollers.

## Current Status

| Metric | Value |
|--------|-------|
| Host unit tests | **942 passing** |
| HIL pytest suite | **116 test functions** (~98% pass rate) |
| Cross-compile targets | STM32F413ZH + STM32G474RE |
| CI | 4 jobs, all green (host tests, cross-check F4/G4, clippy) |
| Crates | 13 |

### Hardware Verified

| Module | NUCLEO-G474RE (FDCAN) | NUCLEO-F413ZH (bxCAN) |
|--------|----------------------|----------------------|
| Clock PLL | 170 MHz boost | 96 MHz HSE bypass |
| DWT timer | PASS | PASS |
| UART | USART2 PA2/PA3 | USART3 PD8/PD9 |
| GPIO LED | PA5 toggle | PA5 toggle |
| CAN loopback | FDCAN PASS | bxCAN PASS |
| CAN real-bus | 4/4 PASS | 4/4 PASS |
| UDS server | 3/3 PASS (13 KB) | 3/3 PASS (13 KB) |
| ISO-TP | SF + multi-frame | SF + multi-frame |
| Scheduler | 1ms/10ms/100ms | 1ms/10ms/100ms |
| Watchdog IWDG | 1s timeout | — |

### Full BSW Stack

| Layer | Crate | Description | Tests |
|-------|-------|-------------|-------|
| BSP | `bsw-bsp-stm32` | Clock, GPIO, UART, CAN, Timer, IWDG | HW verified |
| CAN | `bsw-can` | Abstract CAN transceiver + bus-off recovery | 45 |
| Transport | `bsw-docan` | ISO 15765 (ISO-TP) codec — SF/FF/CF/FC | 52 |
| Diagnostics | `bsw-uds` | ISO 14229 — 9 UDS services | 36 |
| DEM | `bsw-com` | Diagnostic Event Manager — DTC lifecycle | 20 |
| E2E | `bsw-com` | CRC-8 + alive counter protection | 17 |
| NvM | `bsw-io` | Flash emulation / persistent storage | 6 |
| Runtime | `bsw-runtime` | Scheduler, lifecycle, task management | 8 |

### UDS Services Implemented

- `0x10` DiagnosticSessionControl (default + extended)
- `0x11` ECUReset
- `0x22` ReadDataByIdentifier (VIN F190, SW version F195)
- `0x27` SecurityAccess
- `0x2E` WriteDataByIdentifier
- `0x31` RoutineControl
- `0x3E` TesterPresent
- `0x14` ClearDiagnosticInformation
- `0x19` ReadDTCInformation

### HIL Test Coverage

116 pytest functions across 8 test modules running on Raspberry Pi against real hardware:
- `test_uds_services.py` — all 9 UDS services
- `test_isotp_framing.py` — SF, FF/CF/FC multi-frame
- `test_dem_dtc.py` — DTC lifecycle and persistence
- `test_e2e.py` — CRC-8 and alive counter
- `test_nvm_persistence.py` — flash read/write across resets
- `test_session_security.py` — session transitions and security access
- `test_negative_paths.py` — NRC handling and edge cases
- `test_stress.py` — rapid-fire and bus-off recovery

## Methodology: Wrap, Replace, Prove

This is **not** a one-shot translation. We follow the incremental strangler-fig pattern:

```
1. Freeze behavior    — tests, golden outputs, protocol traces
2. Bridge one module  — cxx/autocxx boundary, C++ still does the work
3. Rewrite inward     — AI drafts logic, human reviews FFI/unsafe
4. Prove equivalence  — old C++ stays as oracle, both run same tests
5. Shrink C++         — remove C++ once Rust passes all checks
6. Repeat
```

### Rules

- **No big-bang rewrites** — one module at a time, system works at every step
- **AI writes logic, not boundaries** — FFI/unsafe blocks need human review + written safety justification
- **Old C++ is the oracle** — Rust must match C++ behavior exactly before C++ is removed
- **Leaf modules first** — pure logic, no HW, no OS, no protocol state machines
- **Every `unsafe` block must have a `// SAFETY:` comment** explaining the invariant

### Workspace Structure

```
openbsw-rust/
├── Cargo.toml              # workspace root (13 crates)
├── crates/
│   ├── bsw-bsp-stm32/     # STM32 BSP (clock, GPIO, UART, CAN, timer, IWDG)
│   ├── bsw-can/            # CAN abstraction + transceiver traits
│   ├── bsw-com/            # DEM (DTC lifecycle) + E2E protection
│   ├── bsw-docan/          # ISO 15765 CAN transport (ISO-TP)
│   ├── bsw-doip/           # Diagnostics over IP (ISO 13400)
│   ├── bsw-estd/           # Embedded std containers
│   ├── bsw-ethernet/       # Ethernet abstraction
│   ├── bsw-io/             # NvM / flash storage
│   ├── bsw-lifecycle/      # Component init/shutdown state machine
│   ├── bsw-runtime/        # Scheduler + task management
│   ├── bsw-transport/      # Transport message framing
│   ├── bsw-uds/            # UDS diagnostics (ISO 14229)
│   └── bsw-util/           # Utility functions
├── tests/
│   └── hil/                # HIL pytest suite (116 tests)
└── docs/
    ├── plans/              # Implementation plans
    ├── lessons-learned/    # BSP bringup lessons
    └── test-evidence/      # Verification reports
```

### Tooling

| Tool | Role |
|------|------|
| `cargo test` | 942 host unit tests |
| `pytest` | 116 HIL tests on Raspberry Pi |
| `clippy` | Lint (pedantic, zero warnings) |
| `cargo check --target thumbv7em-none-eabihf` | Cross-compile verification |
| GitHub Actions | CI: host tests + cross-check F4/G4 + clippy |

## Target Hardware

| Board | MCU | CAN | Clock | Flash/RAM |
|-------|-----|-----|-------|-----------|
| NUCLEO-G474RE | STM32G474RE (Cortex-M4F) | FDCAN | 170 MHz | 512K / 128K |
| NUCLEO-F413ZH | STM32F413ZH (Cortex-M4) | bxCAN | 96 MHz | 1.5M / 320K |

All BSP drivers are register-level — no vendor HAL dependency.

## Origin

Based on [Eclipse OpenBSW](https://github.com/eclipse-openbsw/openbsw) (Apache-2.0).
C++ STM32 platform port: [eclipse-openbsw/openbsw#394](https://github.com/eclipse-openbsw/openbsw/issues/394).

## License

Apache-2.0 (matching upstream OpenBSW)
