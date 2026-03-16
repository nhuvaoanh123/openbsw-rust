// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! BSW-stack UDS server on NUCLEO-F413ZH — real CAN bus via crate-level stack.
//!
//! This is the F413ZH port of `bsw_stack_g474.rs`.  The full crate-level BSW
//! stack is assembled identically; only the hardware layer differs:
//!
//! ```text
//!  bxCAN1 HW  <-->  BxCanTransceiver  (bsw-bsp-stm32::can_bxcan)
//!                         |
//!              DiagCanTransport::poll()  (bsw-bsp-stm32::diag_can)
//!                         |
//!              DiagRouter::dispatch()   (bsw-uds::diag_job)
//!                         |
//!               +---------+-----------+----------+
//!               |                     |          |
//!        TesterPresent  DiagnosticSessionControl  ReadDidHandler
//!          (bsw-uds)       (bsw-uds)           (custom, local)
//! ```
//!
//! # Differences from `bsw_stack_g474.rs`
//!
//! | Aspect              | G474RE (FDCAN)                   | F413ZH (bxCAN)                    |
//! |---------------------|----------------------------------|-----------------------------------|
//! | Transceiver type    | `FdCanTransceiver`               | `BxCanTransceiver`                |
//! | Clock module        | `clock_g4::configure_clocks_g474`| `clock_f4::configure_clocks_f413` |
//! | System clock        | 170 MHz                          | 96 MHz                            |
//! | APB1 clock          | 85 MHz                           | 48 MHz                            |
//! | UART module         | `uart_g4::PolledUart`            | `uart_f4::PolledUart`             |
//! | UART init           | `uart.init()` (safe)             | `unsafe { uart.init() }`          |
//! | UART flush          | `uart.flush()`                   | blocking TX — no flush needed     |
//! | CAN GPIO            | PA11/PA12 AF9 (GPIOA AHB2)      | PD0/PD1 AF9 (GPIOD AHB1)         |
//! | RCC AHB GPIO offset | AHB2ENR @ +0x4C                  | AHB1ENR @ +0x30                   |
//! | RCC APB CAN offset  | APB1ENR1 @ +0x58                 | APB1ENR  @ +0x40                  |
//! | Kernel clock select | RCC_CCIPR bits [25:24]           | not applicable (bxCAN uses APB1)  |
//! | GPIOA base          | 0x4800_0000 (AHB2 bus)           | 0x4002_0000 (AHB1 bus)            |
//! | GPIOD base          | n/a                              | 0x4002_0C00                       |
//! | CAN base            | 0x4000_6400 (FDCAN1)             | 0x4000_6400 (bxCAN1, same addr)   |
//! | FIFO drain call     | `isr_rx_fifo0()`                 | `receive_isr()`                   |
//! | VIN                 | `TAKTFLOW_G474_01`               | `TAKTFLOW_F413_01`                |
//!
//! # Lifecycle
//!
//! `DiagCanTransport` implements [`bsw_lifecycle::LifecycleComponent`].
//! Its `init()` method calls `BxCanTransceiver::init()` then `open()`,
//! bringing the hardware bus up.  The main loop calls `transport.poll()`
//! and `transport.transceiver_mut().receive_isr()` on every iteration to
//! drain the bxCAN hardware FIFO and process frames.
//!
//! # Services
//!
//! | SID  | Service                  | Positive response SID |
//! |------|--------------------------|-----------------------|
//! | 0x3E | TesterPresent            | 0x7E                  |
//! | 0x10 | DiagnosticSessionControl | 0x50                  |
//! | 0x22 | ReadDataByIdentifier     | 0x62                  |
//!
//! # DIDs
//!
//! | DID    | Content          | Size                                    |
//! |--------|------------------|-----------------------------------------|
//! | 0xF190 | VIN              | 16 bytes → multi-frame (19 bytes total) |
//! | 0xF195 | SW version       | 9 bytes  → multi-frame (12 bytes total) |
//!
//! # CAN IDs
//!
//! | Direction        | CAN ID |
//! |------------------|--------|
//! | Tester → Server  | 0x600  |
//! | Server → Tester  | 0x601  |
//!
//! # Hardware (NUCLEO-F413ZH)
//!
//! ```text
//! PD8   → USART3_TX  (115200 8N1, routed to ST-LINK VCP)
//! PD9   → USART3_RX
//! PA5   → LD2 (green LED, active-high)
//! PD0   → bxCAN1_RX, AF9 → TJA transceiver RXD
//! PD1   → bxCAN1_TX, AF9 → TJA transceiver TXD
//! ```
//!
//! # GPIO / RCC setup
//!
//! `BxCanTransceiver::init()` configures bxCAN1 bit-timing, filters, and
//! calls `configure_gpio()` / `enable_can_clock()` internally.  This example
//! enables the GPIOA clock (for the LED) before constructing the transceiver,
//! because `uart_f4::PolledUart::init()` handles only the UART GPIO (GPIOD).
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32f413 --target thumbv7em-none-eabihf \
//!             --example bsw_stack_f413
//! ```
//!
//! # Reference
//!
//! RM0402 Rev 8, §32 (bxCAN); ISO 15765-2:2016 (ISO-TP); ISO 14229-1:2020 (UDS).

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::Peripherals as CorePeripherals;

use bsw_bsp_stm32::clock_f4::configure_clocks_f413;
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_f4::PolledUart;
use bsw_bsp_stm32::can_bxcan::BxCanTransceiver;
use bsw_bsp_stm32::diag_can::DiagCanTransport;

use bsw_lifecycle::LifecycleComponent as _;
use bsw_uds::diag_job::{DiagJob, DiagRouter};
use bsw_uds::nrc::Nrc;
use bsw_uds::services::{TesterPresent, DiagnosticSessionControl};
use bsw_uds::session::{DiagSession, SessionMask};

// ---------------------------------------------------------------------------
// Register-map constants — STM32F413ZH (RM0402)
// ---------------------------------------------------------------------------

/// RCC base address (RM0402 §6.3).
const RCC_BASE: usize = 0x4002_3800;
/// RCC AHB1ENR — GPIO clock enables (RM0402 §6.3.9).
const RCC_AHB1ENR_OFFSET: usize = 0x30;

/// GPIOA clock enable (AHB1ENR bit 0).
const GPIOAEN: u32 = 1 << 0;

/// GPIOA base address (RM0402 §8.4).
const GPIOA_BASE: usize = 0x4002_0000;

/// GPIO register offsets (identical layout across all STM32F4 GPIOs).
const GPIO_MODER_OFFSET:   usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_BSRR_OFFSET:    usize = 0x18;

/// LED pin: PA5 (LD2, green, active-high).
const LED_PIN: u32 = 5;

/// Physical request CAN ID: tester → ECU.
const REQUEST_ID: u32 = 0x600;
/// Physical response CAN ID: ECU → tester.
const RESPONSE_ID: u32 = 0x601;

/// Heartbeat print interval: 5 seconds in microseconds.
const HEARTBEAT_US: u64 = 5_000_000;

// ---------------------------------------------------------------------------
// Raw MMIO helpers
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn reg_read(base: usize, offset: usize) -> u32 {
    // SAFETY: caller guarantees valid MMIO address.
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline(always)]
unsafe fn reg_write(base: usize, offset: usize, val: u32) {
    // SAFETY: caller guarantees valid MMIO address.
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

#[inline(always)]
unsafe fn reg_modify(base: usize, offset: usize, mask: u32, bits: u32) {
    let v = unsafe { reg_read(base, offset) };
    unsafe { reg_write(base, offset, (v & !mask) | bits) };
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

/// Configure a GPIO pin as push-pull output, medium-speed, no pull.
unsafe fn gpio_output(base: usize, pin: u32) {
    // SAFETY: base + offsets are valid GPIO MMIO on STM32F4.
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,   0b11 << (pin * 2), 0b01 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
    }
}

/// Set or clear a GPIO pin via BSRR.
unsafe fn gpio_set(base: usize, pin: u32, high: bool) {
    // SAFETY: base + BSRR is valid GPIO MMIO.
    let bit = if high { 1 << pin } else { 1 << (pin + 16) };
    unsafe { reg_write(base, GPIO_BSRR_OFFSET, bit) };
}

// ---------------------------------------------------------------------------
// LED / GPIOA clock enable
//
// BxCanTransceiver::init() enables GPIOD (for PD0/PD1) and CAN1 clocks
// internally.  GPIOA (for the LED on PA5) must be enabled here because
// uart_f4::PolledUart::init() configures only GPIOD for the UART pins.
// ---------------------------------------------------------------------------

/// Enable GPIOA clock and configure PA5 as push-pull output (LED).
///
/// # Safety
///
/// Valid only on STM32F413ZH; must be called once at startup.
unsafe fn led_gpio_init() {
    // SAFETY: RCC and GPIOA addresses are valid on STM32F413ZH.
    unsafe {
        reg_modify(RCC_BASE, RCC_AHB1ENR_OFFSET, 0, GPIOAEN);
        let _ = reg_read(RCC_BASE, RCC_AHB1ENR_OFFSET); // read-back delay
        gpio_output(GPIOA_BASE, LED_PIN);
        gpio_set(GPIOA_BASE, LED_PIN, false);
    }
}

// ---------------------------------------------------------------------------
// Custom ReadDataByIdentifier handler
// ---------------------------------------------------------------------------

/// UDS ReadDataByIdentifier (0x22) handler for DIDs 0xF190 and 0xF195.
///
/// DIDs:
/// - **0xF190** VIN  — 16 ASCII bytes → total response 19 bytes (FF + CFs).
/// - **0xF195** SW version — 9 ASCII bytes → total response 12 bytes (FF + CFs).
struct ReadDidHandler;

impl DiagJob for ReadDidHandler {
    /// Match any request that starts with SID 0x22.
    fn implemented_request(&self) -> &[u8] {
        &[0x22]
    }

    fn session_mask(&self) -> SessionMask {
        SessionMask::ALL
    }

    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        // Minimum: SID(1) + DID_HI(1) + DID_LO(1) = 3 bytes.
        if request.len() < 3 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }

        let did = u16::from_be_bytes([
            request.get(1).copied().unwrap_or(0),
            request.get(2).copied().unwrap_or(0),
        ]);

        match did {
            0xF190 => {
                // VIN — 16-char ASCII: response = [0x62, 0xF1, 0x90, <16 bytes>] = 19 bytes.
                if response.len() < 19 {
                    return Err(Nrc::ResponseTooLong);
                }
                response[0] = 0x62;
                response[1] = 0xF1;
                response[2] = 0x90;
                let vin = b"TAKTFLOW_F413_01";
                response[3..3 + 16].copy_from_slice(vin);
                Ok(19)
            }
            0xF195 => {
                // SW version — 9-char ASCII: response = [0x62, 0xF1, 0x95, <9 bytes>] = 12 bytes.
                if response.len() < 12 {
                    return Err(Nrc::ResponseTooLong);
                }
                response[0] = 0x62;
                response[1] = 0xF1;
                response[2] = 0x95;
                let ver = b"BSP-0.1.0";
                response[3..3 + 9].copy_from_slice(ver);
                Ok(12)
            }
            _ => Err(Nrc::RequestOutOfRange),
        }
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();

    // ------------------------------------------------------------------
    // 1. Clocks — 96 MHz via HSE 8 MHz bypass + PLL
    // ------------------------------------------------------------------
    let sys_clock = configure_clocks_f413();

    // ------------------------------------------------------------------
    // 2. DWT cycle-counter timer
    // ------------------------------------------------------------------
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // ------------------------------------------------------------------
    // 3. UART — USART3 115200 8N1 on PD8 (TX) / PD9 (RX), AF7
    //
    // uart_f4::PolledUart::init() is unsafe (raw MMIO).  F4 PolledUart
    // is blocking — writes stall until the shift register is free, so
    // no explicit flush() call is needed.
    // ------------------------------------------------------------------
    let mut uart = PolledUart::new();
    unsafe { uart.init() };
    let _ = writeln!(
        uart,
        "\r\n=== BSW stack UDS server (F413ZH, real bus) @ {} MHz ===",
        sys_clock / 1_000_000
    );

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    //
    // uart_f4::PolledUart::init() enables GPIOD for the UART pins.
    // GPIOA (LED) must be enabled separately before use.
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Configuring LED (PA5) ...");
    unsafe { led_gpio_init() };

    // ------------------------------------------------------------------
    // 5. Build the BSW stack in main — lifetimes are 'static (main → !)
    //
    // Allocation order:
    //   a. Job objects            — owned by main, live forever.
    //   b. jobs slice             — borrows job references.
    //   c. DiagRouter             — borrows jobs slice.
    //   d. BxCanTransceiver       — owns hardware state, no lifetimes.
    //   e. DiagCanTransport<'a>   — borrows DiagRouter<'a>.
    //
    // BxCanTransceiver::init() (called inside DiagCanTransport::init())
    // enables GPIOD for PD0/PD1, enables the CAN1 APB1 clock, programs
    // bit timing for 500 kbit/s @ 48 MHz APB1, and configures an
    // accept-all receive filter.  No manual GPIO/RCC setup is required
    // here — this differs from the FDCAN path in bsw_stack_g474.rs which
    // required an explicit fdcan1_gpio_rcc_init() call.
    // ------------------------------------------------------------------

    // 5a. Instantiate job objects.
    let tester_present = TesterPresent { session_mask: SessionMask::ALL };
    let session_ctrl   = DiagnosticSessionControl { current_session: DiagSession::Default };
    let read_did       = ReadDidHandler;

    // 5b. Build the jobs slice (array of trait-object references).
    let jobs: [&dyn DiagJob; 3] = [
        &tester_present,
        &session_ctrl,
        &read_did,
    ];

    // 5c. DiagRouter borrows the jobs slice.
    let router = DiagRouter::new(&jobs);

    // 5d. BxCanTransceiver — DO NOT call init()/open() here; DiagCanTransport
    //     does it in its LifecycleComponent::init() implementation.
    let transceiver = BxCanTransceiver::new(0);

    // 5e. DiagCanTransport wraps transceiver + router.
    let mut transport = DiagCanTransport::new(
        transceiver,
        REQUEST_ID,
        RESPONSE_ID,
        router,
    );

    // ------------------------------------------------------------------
    // 6. Lifecycle: init (enters bxCAN1 init mode, programs bit timing,
    //    configures accept-all filter, leaves init mode → bus open).
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Calling DiagCanTransport::init() ...");
    transport.init(); // LifecycleComponent::init() — opens CAN bus

    let _ = writeln!(uart, "BSW stack ready.  Listen=0x{:03X}  Reply=0x{:03X}", REQUEST_ID, RESPONSE_ID);
    let _ = writeln!(uart, "Jobs: TesterPresent(0x3E), DiagnosticSessionControl(0x10), ReadDID(0x22)");
    // F4 PolledUart is blocking — no flush() needed.

    // Solid LED = stack is up and bus is open.
    unsafe { gpio_set(GPIOA_BASE, LED_PIN, true) };

    // ------------------------------------------------------------------
    // 7. Main loop
    //
    //    - receive_isr()   — drain bxCAN1 hardware FIFO0 into the
    //                        software ring buffer.  Must be called each
    //                        loop iteration in polled mode (no ISR).
    //                        Uses `receive_isr()`, not `isr_rx_fifo0()`
    //                        (that is the FDCAN method name).
    //    - transport.poll() — ISO-TP reassembly, UDS dispatch, ISO-TP
    //                         response TX.  Reads from the ring buffer
    //                         via CanReceiver::receive() → dequeue_frame().
    //    - DWT timer read  — LED heartbeat and periodic UART status.
    //    - LED heartbeat   — 1 Hz blink when idle.
    //    - UART status     — every 5 seconds.
    // ------------------------------------------------------------------
    let mut last_heartbeat = timer.system_time_us_64();
    let mut tick: u32 = 0;

    loop {
        // ── Step 0: Drain bxCAN1 hardware RX FIFO into software buffer ──
        // Without this call the software ring buffer is never populated
        // (in polled mode there is no ISR to do it automatically).
        // BxCanTransceiver uses receive_isr() — NOT isr_rx_fifo0() which
        // is the FdCanTransceiver method name.
        unsafe { transport.transceiver_mut().receive_isr() };

        // ── Step 1: Drive the transport (ISO-TP + UDS) ─────────────────
        transport.poll();

        // ── Step 2: LED heartbeat (1 Hz toggle) ────────────────────────
        let now = timer.system_time_us_64();
        let led = ((now / 1_000_000) & 1) != 0;
        unsafe { gpio_set(GPIOA_BASE, LED_PIN, led) };

        // ── Step 3: Periodic UART status every 5 seconds ───────────────
        if now.wrapping_sub(last_heartbeat) >= HEARTBEAT_US {
            last_heartbeat = now;
            tick = tick.wrapping_add(1);
            let _ = writeln!(uart, "heartbeat tick={}", tick);
            // F4 PolledUart is blocking — no flush() needed.
        }
    }
}

// ---------------------------------------------------------------------------
// Panic handler
// ---------------------------------------------------------------------------

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}
