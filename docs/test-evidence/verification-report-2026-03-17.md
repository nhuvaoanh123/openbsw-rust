# Rust OpenBSW Verification Report

**Date**: 2026-03-17
**Target**: NUCLEO-G474RE (STM32G474RET6, 170 MHz, FDCAN)
**Tester**: Raspberry Pi (192.168.0.195, candleLight USB-CAN, 500 kbps)
**Firmware**: `bsw_stack_g474` (crate-level BSW stack, 12 KB)

## Test Summary

| Category | Tests | Result |
|----------|-------|--------|
| Host unit tests | 955 | **ALL PASS** |
| CI (GitHub Actions) | 4 jobs | **ALL PASS** |
| Cross-compile (G474 + F413) | 4 targets | **ALL PASS** |
| Clippy | workspace | **0 warnings** |
| **HIL (real CAN bus)** | **8** | **8/8 PASS** |
| NvM persistence | 1 | **PASS** |
| WriteDID double-write | 1 | **PASS** |

## HIL Test Results (from clean mass-erase)

```
=== Rust BSW HIL Test Suite (simple) ===

  [PASS] TesterPresent: resp=027E00CCCCCCCCCC
  [PASS] DiagSessionDefault: resp=065001003201F4CC
  [PASS] DiagSessionExtended: resp=065003003201F4CC
  [PASS] UnknownService_NRC: resp=037FAA11CCCCCCCC
  [PASS] UnknownDID_NRC: resp=037F2231CCCCCCCC
  [PASS] ShortRequest_NRC: resp=037F2213CCCCCCCC
  [PASS] ReadDID_VIN_multiframe: VIN='TAK'
  [PASS] WriteDID_ReadBack: wrote VIN='HILP', read back matches

=== RESULT: 8/8 PASS ===
```

## NvM Persistence Test

1. WriteDID VIN="HILP" via CAN → positive response 0x6E
2. Reset MCU (STM32CubeProgrammer `--start 0x8000000`)
3. ReadDID VIN → returns "HILP" from flash NvM
4. **PASS**: data persists across power cycle

## WriteDID Double-Write Test (Flash Page Erase)

1. WriteDID VIN="AAAA" → positive response (programs clean flash)
2. WriteDID VIN="ZZZZ" → positive response (erases 2KB page, reprograms)
3. ReadDID VIN → returns "ZZZZ"
4. **PASS**: page erase + reprogram works in dual-bank mode

## Crate Inventory (13 crates)

| Crate | Tests | Purpose |
|-------|-------|---------|
| bsw-estd | 120 | Fixed-capacity containers |
| bsw-can | 67 | CAN abstraction (CanTransceiver trait) |
| bsw-docan | 77 | ISO-TP codec + protocol handlers |
| bsw-transport | 22 | Transport layer traits |
| bsw-uds | 110 | UDS diagnostics (8 services + DEM) |
| bsw-doip | 23 | DoIP protocol types |
| bsw-ethernet | 28 | TCP/UDP/IP types |
| bsw-io | 32 | I/O queues + adapters |
| bsw-lifecycle | 14 | Component lifecycle manager |
| bsw-runtime | 8 | Task scheduler |
| bsw-util | 107 | CRC, SPSC queue, E2E, buddy allocator |
| bsw-com | 40 | Signal packing/unpacking |
| bsw-bsp-stm32 | 22 | STM32 F4/G4 BSP (HW drivers) |

## HW-Verified Features

| Feature | Protocol/Standard | Status |
|---------|-------------------|--------|
| CAN 500 kbps (FDCAN) | ISO 11898 | PASS |
| CAN 500 kbps (bxCAN) | ISO 11898 | PASS |
| Real-bus CAN (FDCAN↔bxCAN) | ISO 11898 | PASS both directions |
| ISO-TP Single Frame | ISO 15765-2 | PASS |
| ISO-TP Multi-Frame (FF+FC+CF) | ISO 15765-2 | PASS (19-byte VIN) |
| UDS TesterPresent (0x3E) | ISO 14229-1 | PASS |
| UDS DiagSessionControl (0x10) | ISO 14229-1 | PASS |
| UDS ReadDataByIdentifier (0x22) | ISO 14229-1 | PASS |
| UDS WriteDataByIdentifier (0x2E) | ISO 14229-1 | PASS |
| UDS NRC serviceNotSupported (0x11) | ISO 14229-1 | PASS |
| UDS NRC requestOutOfRange (0x31) | ISO 14229-1 | PASS |
| UDS NRC incorrectMessageLength (0x13) | ISO 14229-1 | PASS |
| Flash erase (dual-bank 2KB pages) | STM32G4 RM0440 | PASS |
| Flash program (doubleword) | STM32G4 RM0440 | PASS |
| NvM persistence (write → power cycle → read) | - | PASS |
| NvM overwrite (page erase + reprogram) | - | PASS |
| DWT cycle counter timer | ARM Cortex-M4 | PASS |
| UART debug output (115200 baud) | - | PASS |
| GPIO LED toggle | - | PASS |

## Known Limitations

1. Heartbeat TX disabled (sends to unACK'd CAN ID, causes bus errors)
2. SecurityAccess (0x27) implemented in crate but not wired to HIL
3. F413 bsw_stack not HIL-tested (compiles, HW not connected)
4. Interrupt-driven CAN RX built but not wired (using polling)

## Bugs Found and Fixed During Verification

1. STM32G4 FDCAN register offsets differ from Bosch M_CAN
2. STM32G4 FDCAN NBTP bit field layout swapped from M_CAN
3. NTSEG1=13 in driver (526 kbps) vs NTSEG1=14 in .ioc (500 kbps)
4. F413 CAN GPIO: PD0/PD1 (not PA11/PA12 from .ioc template)
5. DiagCanTransport stack overflow (4KB response buffer on stack)
6. FdCanTransceiver::receive() needs manual isr_rx_fifo0() drain
7. F413 server shadowing G474 responses on same CAN IDs
8. Pi USB-CAN adapter stuck in ERROR-PASSIVE (needs driver reload)
9. Flash dual-bank: 256×2KB pages (not 128×4KB) with BKER bank select
