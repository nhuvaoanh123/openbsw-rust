# Lessons Learned — Rust BSP Bringup (openbsw-rust)

## 2026-03-15 — Three hardware-config bugs on STM32 NUCLEO boards

**Context**: First hardware test of pure Rust BSP (`bsw-bsp-stm32`) on NUCLEO-G474RE and NUCLEO-F413ZH. Code compiled clean for both features. DWT timer, state machines, CAN drivers all type-checked. LED blink loop confirmed running via SWD register read.

**Bug 1 — G4 PLL divider encoding**: STM32G4 RCC_PLLCFGR PLLR bits [26:25] use encoding `00=/2, 01=/4, 10=/6, 11=/8` — NOT `value-1`. Code had `(2-1) << 25 = 1 << 25`, which set PLLR to /4 instead of /2. SYSCLK was 85 MHz instead of 170 MHz. UART output was garbled at exactly half the expected baud rate.
**Fix**: Changed to `0b00 << 25` for /2. Confirmed via SWD register readback.
**Principle**: STM32 PLL divider registers use inconsistent encodings — some are `value-1` (PLLM, PLLN), some are enumerated (PLLP, PLLR). Always verify the encoding table in the reference manual for each field individually. Don't assume a pattern.

**Bug 2 — G4 UART missing GPIO alternate function config**: UART init enabled the peripheral clock and configured USART2 registers (BRR, CR1) but did not configure PA2/PA3 as AF7. Pins were in default analog mode → no signal on the wire. USART2 registers read back correctly via SWD (SR=0xC0, BRR=0x5C4, CR1 correct), but no output because GPIO was wrong.
**Fix**: Added GPIOA clock enable + MODER/OSPEEDR/AFRL configuration for PA2/PA3 AF7 in `init()`.
**Principle**: When UART registers look correct but no output appears, check GPIO configuration first. USART will happily "transmit" into an unconfigured pin. Always configure GPIO AF before enabling the peripheral.

**Bug 3 — F4 UART wrong USART peripheral**: NUCLEO-F413ZH (144-pin) routes ST-LINK VCP to **USART3 on PD8/PD9**, not USART2 on PA2/PA3. The 64-pin NUCLEO boards (like G474RE) use USART2/PA2/PA3 for VCP. Code was using USART2 — registers configured correctly but output went to Arduino connector pins, not to VCP.
**Fix**: Changed from USART2/PA2/PA3 to USART3/PD8/PD9 (AF7). PD8/PD9 use AFRH (not AFRL).
**Principle**: NUCLEO board VCP routing depends on package size. 64-pin → USART2/PA2/PA3. 144-pin → USART3/PD8/PD9. Always check the board schematic/user manual (UM1974/UM2010) for VCP wiring, don't assume USART2.

**Debugging approach that worked**: (1) Read core registers via SWD to confirm MCU is running and where PC is. (2) Read peripheral registers (RCC_PLLCFGR, USART_SR/BRR/CR1, GPIO_MODER/AFRL) to verify configuration. (3) Calculate actual frequencies from register values. The SWD register dump immediately identified PLLR=01 → /4 and showed UART was configured but GPIO wasn't.

## 2026-03-16 — DWT timer integer division truncation kills scheduler + watchdog

**Context**: Full Rust BSW app on G474RE with DWT-based system timer, cooperative scheduler (1ms/10ms/100ms tasks), and IWDG watchdog kicked from 1ms task. Watchdog kept resetting the board every 1 second despite scheduler appearing to work.

**Mistake**: `DwtTimer::update()` computed `ticks_us += elapsed_cycles / freq_mhz`. At 170 MHz, `freq_mhz = 170`. In a tight main loop, the DWT cycle counter advances only ~68 cycles between calls. `68 / 170 = 0` (integer division truncation). The microsecond accumulator stopped advancing in tight loops, so no scheduler tasks ever became "due", and the watchdog was never kicked.

**Why it appeared to work in the blink example**: The blink example used `system_time_us_64()` with a simple LED toggle — the loop body was heavier (volatile GPIO write + branch), causing DWT to advance ~200+ cycles between calls, just enough for `200/170 = 1` to not truncate to zero.

**Fix**: Track remainder cycles: `total_cycles = elapsed + remainder; ticks_us += total/freq_mhz; remainder = total % freq_mhz`. This ensures sub-microsecond cycle fractions accumulate correctly across calls.

**Principle**: Never divide a monotonic counter by a fixed rate without handling the remainder. Integer division truncation in cycle-to-time conversion is invisible until the loop body is faster than one time quantum (1us at MHz-scale). Always track the remainder, or accumulate raw cycles and convert lazily.

**Also fixed**: IWDG init sequence — must write `KEY_ENABLE` (0xCCCC) before `KEY_UNLOCK` (0x5555) to start the LSI oscillator; otherwise the PVU/RVU wait for prescaler/reload register update hangs because LSI isn't clocked.

## 2026-03-16 — STM32G4 FDCAN uses different register map from Bosch M_CAN

**Context**: FDCAN CAN loopback test on NUCLEO-G474RE — init succeeded, TX frame sent, but RX FIFO 0 stayed empty for 500ms timeout. The driver was a direct Rust translation of the C++ `FdCanDevice.cpp` which used generic M_CAN register offsets.

**Mistake**: Assumed the STM32G4 FDCAN uses the same register layout as the generic Bosch M_CAN IP core. It doesn't. ST implemented a **simplified** FDCAN for the G4 family with:
1. **Shifted register offsets**: RXF0S at 0x090 (not 0x0A4), TXFQS at 0x0C4 (not 0x0C8), TXBAR at 0x0CC (not 0x0D0), TXBC at 0x0C0 (not 0x0C4).
2. **Non-existent registers**: SIDFC (0x084), XIDFC (0x088), RXF0C (0x0A0), RXF1C (0x0B0), TXEFC (0x0F0) — these are XIDAM, HPMS, reserved, reserved, TXEFA on G4. Writing to them corrupted other registers (XIDAM) or was silently ignored.
3. **Hardware-fixed MRAM layout**: no configurable start-address registers. Each FDCAN1 section has a fixed offset from SRAMCAN_BASE.
4. **18-word (72-byte) element stride**: RX/TX elements are 18 words each (FD-size), not 4 words (classic CAN size). TX data written at wrong MRAM offset, RX data read from wrong offset.
5. **Narrower bit fields**: RXF0S fill-level is 4 bits [3:0] (not 7), get-index is 2 bits [9:8] (not 6 bits).

**Symptoms**: TXBAR write at 0x0D0 → hit wrong register, TX never requested. RXF0S read at 0x0A4 → reserved, always 0 → "no frames received". SIDFC write at 0x084 → corrupted XIDAM. Register dump showed TXBC=0x00000003 (reading TXFQS at the wrong offset).

**Fix**: Updated all register offsets to match STM32G4 CMSIS `FDCAN_GlobalTypeDef`. Removed writes to non-existent registers. Changed MRAM element stride from 16 → 72 bytes. Added TXBC configuration (reset value is 0 = no TX buffers).

**Porting rule**: Every time you port FDCAN to a new STM32 family, verify: (1) register offsets against CMSIS `FDCAN_GlobalTypeDef` struct, (2) which config registers exist (SIDFC/XIDFC/RXF0C may not), (3) MRAM element stride, (4) bit field widths in status registers. Start from the target chip's CMSIS/PAC headers, never from a "generic" M_CAN spec.

**Principle**: Never assume peripheral register maps are portable across MCU families, even within the same vendor. The C++ code had the same bug — it compiled fine because register offsets are just integer constants. Always validate with a diagnostic register dump on real hardware.

## 2026-03-16 — STM32G4 FDCAN NBTP bit field layout swapped from M_CAN

**Context**: Real-bus CAN test between G474RE (FDCAN) and F413ZH (bxCAN). Both at 500 kbps. F413 tester transmitted successfully (TXOK0=1), Pi saw frames, but G474 FDCAN went bus-off with Bit 0/1 Errors or stayed in "synchronizing" (ACT=1).

**Mistake**: Used standard M_CAN NBTP bit field layout: `[31:25]=NSJW, [24:16]=NTSEG1, [14:8]=NTSEG2, [8:0]=NBRP`. STM32G4 FDCAN has **swapped** fields: `[31:25]=NSJW, [24:16]=NBRP, [15:8]=NTSEG1, [6:0]=NTSEG2`. NBRP and NTSEG1 are in each other's positions.

**Symptom**: NBTP register value 0x060E0310 decoded as prescaler=15 (wrong), Tseg1=4 (wrong), Tseg2=17 (wrong) → ~515 kbps actual baud. The 3% mismatch from the bus's 500 kbps caused the FDCAN to detect bit errors on every frame, driving TEC to 255 (bus-off).

**Diagnostic trail**: (1) F413 TXOK0=1 + Pi candump saw frames → bus works. (2) G474 PSR ACT=1 with ECR=0 → FDCAN never synchronized when idle bus. (3) After Pi sends frames → G474 PSR LEC=2 (Bit 0 Error), TEC=255, bus-off → sees traffic but can't decode. (4) GPIO/clock dump confirmed PA11/PA12 AF9 and PCLK1 correct. (5) Checked CMSIS header → NBRP_Pos=16, NTSEG1_Pos=8 — swapped!

**Fix**: Changed NBTP encoding from `(nsjw<<25)|(ntseg1<<16)|(ntseg2<<8)|nbrp` to `(nsjw<<25)|(nbrp<<16)|(ntseg1<<8)|ntseg2`. Result: 0x06100E03 → correct 500 kbps → 4/4 UDS tests PASS both directions.

**Also fixed**: F413 CAN GPIO was PA11/PA12 (from .ioc template) but production wiring is PD0/PD1 (from rzc_f4_hw_stm32f4.c). Always check the actual hardware platform C file, not just the .ioc.

**Updated porting rule**: When porting FDCAN to a new STM32 family, also verify (5) NBTP/DBTP bit field positions — they may be swapped. The STM32G4 FDCAN deviates from M_CAN in at least: register offsets, MRAM config presence, element stride, status field widths, AND bit timing register layout.

## 2026-03-16 — Pi CAN interface as bus node: must be up before testing

**Context**: Two-board CAN test failed with bus-off on both nodes. Pi's can0 was on the same physical bus and provided one 120Ω termination.

**Root cause**: Pi's can0 was already UP and healthy (ERROR-ACTIVE at 500 kbps). The actual issue was the F413 using wrong GPIO pins (PA11/PA12 instead of PD0/PD1).

**Lesson**: When debugging CAN bus issues with multiple nodes, use the Pi as a known-good reference: `cansend can0 600#023E00` to inject test frames, `candump can0` to sniff. If the Pi can send (gets ACK) but a board can't receive, the board's RX path is broken. If the Pi sees frames from one board but another board doesn't, the second board isn't on the bus.

## 2026-03-16 — Multiple UDS servers on same CAN ID pair cause response shadowing

**Context**: WriteDID (0x2E) returned NRC 0x11 (serviceNotSupported) on G474, despite the handler being registered in DiagRouter. TesterPresent (0x3E) worked. Investigation explored vtable corruption, LTO issues, router dispatch bugs — all dead ends.

**Root cause**: The F413ZH was still running `can_server_f413` firmware on the same bus, listening on the same CAN IDs (0x600/0x601). When the Pi sent a WriteDID request, BOTH the G474 and F413 received it. The F413 responded first with NRC 0x11 (it doesn't support 0x2E). The Pi's `candump` captured the F413's response and ignored the G474's (which arrived later and was lost in the bus arbitration).

**Why TesterPresent worked**: Both servers support 0x3E — the F413 responded with `7E 00` and the G474 also responded with `7E 00`. Since both responses are identical, it appeared to work.

**How we found it**: (1) Bypassed the DiagRouter entirely (always return positive) — still got NRC. (2) Defined handler IN the BSP crate (same compilation unit) — still NRC. (3) Used handler as ONLY job — still NRC. (4) Checked F413 UART — found it actively processing and responding to requests.

**Fix**: Erase or disable CAN on all boards except the one under test. On a multi-ECU bench, each ECU must use unique CAN ID pairs for UDS (e.g., CVC=0x641/0x642, FZC=0x643/0x644).

**Principle**: When a UDS response is unexpected (wrong NRC, wrong data), check ALL nodes on the bus — another ECU may be responding to the same request ID. CAN has no source address in the frame — you can't tell which node sent a response by looking at the CAN ID alone. Use unique request/response ID pairs per ECU, or ensure only one server is active at a time during testing.
