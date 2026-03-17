// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! BSW-stack UDS server on NUCLEO-G474RE — real CAN bus via crate-level stack.
//!
//! Unlike `can_server_g474.rs`, which implements ISO-TP and UDS dispatch
//! inline, this example assembles the full crate-level BSW stack:
//!
//! ```text
//!  FDCAN1 HW  <-->  FdCanTransceiver  (bsw-bsp-stm32::can_fdcan)
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
//! # Lifecycle
//!
//! `DiagCanTransport` implements [`bsw_lifecycle::LifecycleComponent`].
//! Its `init()` method calls `FdCanTransceiver::init()` then `open()`,
//! bringing the hardware bus up.  The main loop calls `transport.poll()`
//! on every iteration to drain and process CAN frames.
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
//! | DID    | Content          | Size     |
//! |--------|------------------|----------|
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
//! # Hardware (NUCLEO-G474RE)
//!
//! ```text
//! PA2   → USART2_TX  (115200 8N1, SB connected to ST-LINK VCP by default)
//! PA3   → USART2_RX
//! PA5   → LD2 (green LED, active-high)
//! PA11  → FDCAN1_RX, AF9 → TJA transceiver RXD
//! PA12  → FDCAN1_TX, AF9 → TJA transceiver TXD
//! ```
//!
//! # GPIO / RCC setup
//!
//! `FdCanTransceiver::init()` configures FDCAN1 bit-timing, message-RAM, and
//! interrupts, but does **not** enable RCC clocks or configure GPIO alternate
//! functions.  This example does that manually (identical to `can_server_g474.rs`)
//! before constructing the transceiver.
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32g474 --target thumbv7em-none-eabihf \
//!             --example bsw_stack_g474
//! ```
//!
//! # Reference
//!
//! RM0440 Rev 8, §44 (FDCAN); ISO 15765-2:2016 (ISO-TP); ISO 14229-1:2020 (UDS).

#![no_std]
#![no_main]

use core::fmt::Write as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::Peripherals as CorePeripherals;

use bsw_bsp_stm32::clock_g4::configure_clocks_g474;
use bsw_bsp_stm32::timer::DwtTimer;
use bsw_bsp_stm32::uart_g4::PolledUart;
use bsw_bsp_stm32::can_fdcan::FdCanTransceiver;
use bsw_can::transceiver::CanTransceiver as _;
use bsw_bsp_stm32::diag_can::DiagCanTransport;
use bsw_bsp_stm32::flash_g4::FlashG4;
use bsw_bsp_stm32::nvm::{NvmManager, NvmBlockId};

use bsw_lifecycle::LifecycleComponent as _;
use bsw_uds::diag_job::{DiagJob, DiagRouter};
use bsw_uds::nrc::Nrc;
use bsw_uds::services::{TesterPresent, DiagnosticSessionControl, ControlDtcSetting, SecurityAccess, RoutineControl};
use bsw_uds::session::{DiagSession, SessionMask};

// ---------------------------------------------------------------------------
// Register-map constants (same as can_server_g474.rs for consistency)
// ---------------------------------------------------------------------------

/// RCC base address (RM0440 §6.4).
const RCC_BASE: usize = 0x4002_1000;
/// RCC AHB2ENR — GPIOA/B/C clock enables.
const RCC_AHB2ENR_OFFSET: usize = 0x4C;
/// RCC APB1ENR1 — FDCAN1 clock is on APB1 (bit 25).
const RCC_APB1ENR1_OFFSET: usize = 0x58;
/// RCC CCIPR — kernel clock selection (bits [25:24] for FDCAN).
const RCC_CCIPR_OFFSET: usize = 0x88;
/// FDCAN1EN bit in APB1ENR1 (bit 25).
const FDCAN1EN: u32 = 1 << 25;
/// GPIOA clock enable (AHB2ENR bit 0).
const GPIOAEN: u32 = 1 << 0;

/// GPIOA base address.
const GPIOA_BASE: usize = 0x4800_0000;

/// GPIO register offsets.
const GPIO_MODER_OFFSET:   usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_AFRL_OFFSET:    usize = 0x20;
const GPIO_AFRH_OFFSET:    usize = 0x24;
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
// Raw MMIO helpers (identical to can_server_g474.rs)
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

/// Configure a GPIO pin as push-pull output, high-speed.
unsafe fn gpio_output(base: usize, pin: u32) {
    // SAFETY: base + offsets are valid GPIO MMIO on STM32G4.
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

/// Configure a GPIO pin as alternate function.
unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
    // SAFETY: base + AF offsets are valid GPIO MMIO on STM32G4.
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,   0b11 << (pin * 2), 0b10 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
        if pin < 8 {
            let shift = pin * 4;
            reg_modify(base, GPIO_AFRL_OFFSET, 0xF << shift, af << shift);
        } else {
            let shift = (pin - 8) * 4;
            reg_modify(base, GPIO_AFRH_OFFSET, 0xF << shift, af << shift);
        }
    }
}

// ---------------------------------------------------------------------------
// FDCAN1 GPIO + clock init
//
// Configure PA11 (FDCAN1_RX) / PA12 (FDCAN1_TX) as AF9 and enable all
// required RCC clocks.  This must be done BEFORE FdCanTransceiver::init().
// ---------------------------------------------------------------------------

/// Enable FDCAN1 APB1 clock, select PCLK1 as kernel clock, and configure
/// PA11 / PA12 as AF9 for FDCAN1.
unsafe fn fdcan1_gpio_rcc_init() {
    // SAFETY: RCC, GPIOA addresses are valid on STM32G474RE.
    unsafe {
        // Enable GPIOA clock.  USART2 init may have done this already; safe
        // to set again — it is OR-in semantics in the enable register.
        reg_modify(RCC_BASE, RCC_AHB2ENR_OFFSET, 0, GPIOAEN);
        let _ = reg_read(RCC_BASE, RCC_AHB2ENR_OFFSET); // read-back delay

        // PA11 = FDCAN1_RX (AF9), PA12 = FDCAN1_TX (AF9).
        gpio_af(GPIOA_BASE, 11, 9);
        gpio_af(GPIOA_BASE, 12, 9);

        // Select PCLK1 as FDCAN kernel clock: CCIPR bits [25:24] = 0b10.
        reg_modify(RCC_BASE, RCC_CCIPR_OFFSET, 0b11 << 24, 0b10 << 24);

        // Enable FDCAN1 peripheral clock.
        reg_modify(RCC_BASE, RCC_APB1ENR1_OFFSET, 0, FDCAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET); // read-back delay
    }
}

// ---------------------------------------------------------------------------
// Custom ReadDataByIdentifier handler
// ---------------------------------------------------------------------------

/// UDS ReadDataByIdentifier (0x22) handler — reads VIN from NvM.
///
/// DIDs:
/// - **0xF190** VIN  — reads from flash NvM (16 bytes), falls back to default
/// - **0xF195** SW version — hardcoded "BSP-0.1.0"
struct ReadDidHandler;

impl DiagJob for ReadDidHandler {
    fn implemented_request(&self) -> &[u8] {
        &[0x22]
    }

    fn session_mask(&self) -> SessionMask {
        SessionMask::ALL
    }

    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        if request.len() < 3 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }

        let did = u16::from_be_bytes([request[1], request[2]]);

        match did {
            0xF190 => {
                // VIN — read from NvM, fallback to default if empty
                response[0] = 0x62;
                response[1] = 0xF1;
                response[2] = 0x90;
                let mut vin_buf = [0u8; 17];
                let nvm_len = NvmManager::read_block(NvmBlockId::Vin, &mut vin_buf);
                if nvm_len > 0 {
                    let len = nvm_len.min(16);
                    response[3..3 + len].copy_from_slice(&vin_buf[..len]);
                    Ok(3 + len)
                } else {
                    let default_vin = b"TAKTFLOW_G474_01";
                    response[3..3 + 16].copy_from_slice(default_vin);
                    Ok(19)
                }
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

/// UDS WriteDataByIdentifier (0x2E) handler — writes VIN to flash NvM.
///
/// Only DID 0xF190 (VIN) is writable. Writes persist across power cycles.
struct WriteDidHandler;

impl DiagJob for WriteDidHandler {
    fn implemented_request(&self) -> &[u8] {
        &[0x2E]
    }

    fn session_mask(&self) -> SessionMask {
        // Allow writes in all sessions for demo. Production: EXTENDED only.
        SessionMask::ALL
    }

    fn process(&self, request: &[u8], response: &mut [u8]) -> Result<usize, Nrc> {
        // WriteDID: [0x2E, DID_hi, DID_lo, data...]
        if request.len() < 4 {
            return Err(Nrc::IncorrectMessageLengthOrInvalidFormat);
        }

        let did = u16::from_be_bytes([request[1], request[2]]);
        let data = &request[3..];

        match did {
            0xF190 => {
                // Write VIN to NvM (max 17 bytes)
                let write_len = data.len().min(17);
                let ok = unsafe {
                    FlashG4::unlock();
                    FlashG4::clear_errors();
                    let result = NvmManager::write_block(NvmBlockId::Vin, &data[..write_len]);
                    // Don't lock — keep flash unlocked for subsequent writes.
                    // Production code should lock after all writes are done.
                    result
                };
                if ok {
                    response[0] = 0x6E; // positive response = SID + 0x40
                    response[1] = 0xF1;
                    response[2] = 0x90;
                    Ok(3)
                } else {
                    Err(Nrc::GeneralProgrammingFailure)
                }
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
    // 1. Clocks — 170 MHz boost mode
    // ------------------------------------------------------------------
    let sys_clock = configure_clocks_g474();

    // ------------------------------------------------------------------
    // 2. DWT cycle-counter timer
    // ------------------------------------------------------------------
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // ------------------------------------------------------------------
    // 3. UART — USART2 115200 8N1 on PA2 (TX) / PA3 (RX)
    //
    // PolledUart::init() enables GPIOA clock and configures PA2/PA3 AF7.
    // ------------------------------------------------------------------
    let mut uart = PolledUart::new();
    uart.init();
    let _ = writeln!(
        uart,
        "\r\n=== BSW stack UDS server (G474RE, real bus) @ {} MHz ===",
        sys_clock / 1_000_000
    );

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    //
    // GPIOA clock was already enabled by PolledUart::init().
    // ------------------------------------------------------------------
    unsafe {
        gpio_output(GPIOA_BASE, LED_PIN);
        gpio_set(GPIOA_BASE, LED_PIN, false);
    }

    // ------------------------------------------------------------------
    // 5. GPIO / RCC setup for FDCAN1
    //
    // FdCanTransceiver::init() handles FDCAN1 mode transitions and bit
    // timing, but NOT GPIO alternate functions or RCC clocks.  Configure
    // those here, before constructing the transceiver.
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Configuring FDCAN1 GPIO (PA11=RX, PA12=TX, AF9) + RCC ...");
    unsafe {
        fdcan1_gpio_rcc_init();
    }

    // ------------------------------------------------------------------
    // 6. Build the BSW stack in main — lifetimes are 'static (main → !)
    //
    // Allocation order:
    //   a. Job objects           — owned by main, live forever.
    //   b. jobs slice            — borrows job references.
    //   c. DiagRouter            — borrows jobs slice.
    //   d. FdCanTransceiver      — owns hardware state, no lifetimes.
    //   e. DiagCanTransport<'a>  — borrows DiagRouter<'a>.
    //
    // Because main() never returns, every local variable here is
    // effectively 'static, satisfying all borrow-checker lifetime
    // requirements without any unsafe transmutes.
    // ------------------------------------------------------------------

    // 6a. Instantiate job objects.
    let tester_present = TesterPresent { session_mask: SessionMask::ALL };
    let session_ctrl   = DiagnosticSessionControl { current_session: DiagSession::Default };
    let read_did       = ReadDidHandler;
    let write_did      = WriteDidHandler;
    let control_dtc    = ControlDtcSetting { session_mask: SessionMask::ALL };
    let security       = SecurityAccess::new(SessionMask::ALL, 3);
    let routine_ctrl   = RoutineControl { session_mask: SessionMask::ALL };

    // 6b. Build the jobs slice (array of trait-object references).
    let jobs: [&dyn DiagJob; 7] = [
        &tester_present,
        &session_ctrl,
        &read_did,
        &write_did,
        &control_dtc,
        &security,
        &routine_ctrl,
    ];

    // 6c. DiagRouter borrows the jobs slice.
    let router = DiagRouter::new(&jobs);

    // 6d. FdCanTransceiver — DO NOT call init()/open() here; DiagCanTransport
    //     does it in its LifecycleComponent::init() implementation.
    let transceiver = FdCanTransceiver::new(0);

    // 6e. DiagCanTransport wraps transceiver + router.
    let mut transport = DiagCanTransport::new(
        transceiver,
        REQUEST_ID,
        RESPONSE_ID,
        router,
    );

    // ------------------------------------------------------------------
    // 7. Lifecycle: init (enters FDCAN1 init mode, sets bit timing,
    //    configures MRAM / RXGFC, leaves init mode → bus open).
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Calling DiagCanTransport::init() ...");
    transport.init(); // LifecycleComponent::init() — opens CAN bus

    // Debug: read back FDCAN registers after init
    unsafe {
        let nbtp = core::ptr::read_volatile(0x4000_641C as *const u32);
        let cccr = core::ptr::read_volatile(0x4000_6418 as *const u32);
        let psr = core::ptr::read_volatile(0x4000_6444 as *const u32);
        let _ = writeln!(uart, "FDCAN: NBTP=0x{:08X} CCCR=0x{:08X} PSR=0x{:08X}", nbtp, cccr, psr);
        let _ = writeln!(uart, "Expected NBTP=0x{:08X}", (3u32 << 25) | (16 << 16) | (13 << 8) | 3);
    }
    // Debug: test flash erase on page 125 (NvM data page, bank 2)
    unsafe {
        let _ = writeln!(uart, "Flash test: unlock...");
        let unlocked = FlashG4::unlock();
        let cr_before = core::ptr::read_volatile(0x4002_2014 as *const u32);
        let _ = writeln!(uart, "  unlock={} CR=0x{:08X}", unlocked, cr_before);

        let _ = writeln!(uart, "Flash test: erase page 125...");
        FlashG4::clear_errors();
        let erased = FlashG4::erase_page(125);
        let sr_after = core::ptr::read_volatile(0x4002_2010 as *const u32);
        let cr_after = core::ptr::read_volatile(0x4002_2014 as *const u32);
        let _ = writeln!(uart, "  erased={} SR=0x{:08X} CR=0x{:08X}", erased, sr_after, cr_after);

        // Flush caches and read page to verify erased (all 0xFF)
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        // Invalidate D-cache for this address range if enabled
        let val = core::ptr::read_volatile(0x0807_D000 as *const u32);
        let val2 = core::ptr::read_volatile(0x0807_D004 as *const u32);
        let _ = writeln!(uart, "  page125[0..7]=0x{:08X} 0x{:08X} (expect FFFFFFFF)", val, val2);

        // Also try erasing WITHOUT BKER to see if that works
        let _ = writeln!(uart, "Flash test: erase page 125 without BKER...");
        FlashG4::clear_errors();
        // Direct CR write: PER=1, PNB=125, BKER=0, STRT=1
        let cr_val = (1u32 << 1) | (125u32 << 3) | (1u32 << 16);
        core::ptr::write_volatile(0x4002_2014 as *mut u32, cr_val);
        while core::ptr::read_volatile(0x4002_2010 as *const u32) & (1 << 16) != 0 {}
        let sr2 = core::ptr::read_volatile(0x4002_2010 as *const u32);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        let val3 = core::ptr::read_volatile(0x0807_D000 as *const u32);
        let _ = writeln!(uart, "  no-BKER: SR=0x{:08X} page125[0]=0x{:08X}", sr2, val3);

        FlashG4::lock();
    }

    let _ = writeln!(uart, "BSW stack ready.  Listen=0x{:03X}  Reply=0x{:03X}", REQUEST_ID, RESPONSE_ID);
    uart.flush();

    // Solid LED = stack is up and bus is open.
    unsafe { gpio_set(GPIOA_BASE, LED_PIN, true) };

    // ------------------------------------------------------------------
    // 8. Main loop
    //
    //    - transport.poll() — drain CAN RX, ISO-TP reassembly, UDS dispatch,
    //      ISO-TP response TX.  One call per loop iteration.
    //    - DWT timer read — used for LED heartbeat and periodic UART status.
    //    - LED heartbeat — 1 Hz blink when idle.
    //    - UART status print — every 5 seconds.
    // ------------------------------------------------------------------
    let mut last_heartbeat = timer.system_time_us_64();
    let mut tick: u32 = 0;

    // Heartbeat: cyclic CAN frame every 100 ms
    // CAN ID 0x010 = CVC/FZC heartbeat (matches production firmware)
    const HEARTBEAT_CAN_ID: u32 = 0x010;
    const HEARTBEAT_PERIOD_US: u64 = 100_000; // 100 ms
    let mut heartbeat_counter: u8 = 0;
    let mut last_hb_tx = timer.system_time_us_64();
    // RX heartbeat monitoring from other ECU
    const PEER_HEARTBEAT_ID: u32 = 0x011; // Other ECU's heartbeat
    const PEER_TIMEOUT_US: u64 = 500_000; // 500 ms timeout
    let mut last_peer_hb = timer.system_time_us_64();
    let mut peer_alive = false;

    loop {
        // ── Step 0: Drain FDCAN1 hardware RX FIFO into software buffer ──
        unsafe { transport.transceiver_mut().isr_rx_fifo0() };

        // ── Step 0b: Heartbeat TX — DISABLED to prevent bus errors ──────
        // The heartbeat sends on 0x010 which nobody ACKs, causing TEC to
        // increment and eventually Error Passive / Bus Off. Re-enable when
        // a peer ECU is listening for heartbeat frames.
        let now_hb = timer.system_time_us_64();
        let delta = now_hb.wrapping_sub(last_hb_tx);
        if false && delta >= HEARTBEAT_PERIOD_US {
            last_hb_tx = now_hb;
            let hb_data = [heartbeat_counter, 0x01, 0x00, 0x00]; // counter + status=running
            let hb_frame = bsw_can::frame::CanFrame::with_data(
                bsw_can::can_id::CanId::base(HEARTBEAT_CAN_ID as u16),
                &hb_data,
            );
            let ec = transport.transceiver_mut().write(&hb_frame);
            // Heartbeat TX fires — frame queued to FDCAN.
            // Note: if FDCAN is in bus-off, write() returns Ok but frame won't
            // actually transmit. The bus-off recovery in poll() handles this.
            heartbeat_counter = heartbeat_counter.wrapping_add(1);
        }

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
            // Print transceiver state for bus-off debugging
            let ts = transport.transceiver().transceiver_state();
            let st = transport.transceiver().state();
            let _ = writeln!(uart, "tick={} bus={:?} state={:?}", tick, ts, st);
            uart.flush();
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
