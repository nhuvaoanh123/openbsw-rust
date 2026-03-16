//! UDS tester on NUCLEO-G474RE — exercises `can_server_g474` over a real CAN bus.
//!
//! This example sends four UDS requests to the server board and verifies the
//! responses.  Both boards must be connected via a TJA105x transceiver with
//! 120 Ω termination at each end of the bus.
//!
//! # Test sequence
//!
//! | # | Service              | Request             | Expected response        |
//! |---|----------------------|---------------------|--------------------------|
//! | 1 | TesterPresent        | `3E 00`             | SF `[02, 7E, 00]`        |
//! | 2 | DiagSessionCtrl ext  | `10 03`             | SF `[07, 50, 03, ...]`   |
//! | 3 | ReadDID unknown      | `22 00 01`          | SF `[04, 7F, 22, 31]`    |
//! | 4 | Unknown service      | `AA`                | SF `[04, 7F, AA, 11]`    |
//!
//! All four tests use Single Frame ISO-TP — no multi-frame handling required
//! in the tester.  This proves real-bus CAN communication end-to-end.
//!
//! # GPIO / wiring (NUCLEO-G474RE)
//!
//! ```text
//! PA11  →  TJA RXD   (FDCAN1_RX, AF9)
//! PA12  →  TJA TXD   (FDCAN1_TX, AF9)
//! GND / VCC as per transceiver module datasheet.
//! CAN_H / CAN_L connected to server board transceiver.
//! 120 Ω termination at each end.
//! ```
//!
//! # CAN IDs
//!
//! | Direction        | CAN ID |
//! |------------------|--------|
//! | Tester → Server  | 0x600  |
//! | Server → Tester  | 0x601  |
//!
//! # Bit timing: 500 kbit/s @ 170 MHz
//!
//! ```text
//! Prescaler = 17 (NBRP=16), Tseg1=15 (NTSEG1=14), Tseg2=4 (NTSEG2=3), SJW=4 (NSJW=3)
//! Bit = 1 + 15 + 4 = 20 TQ → 170 MHz / (17 × 20) = 500 000 bit/s ✓
//! ```
//!
//! # LED indication
//!
//! * Fast blink (100 ms) — all 4 tests passed.
//! * Slow blink (500 ms) — one or more tests failed.
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32g474 --target thumbv7em-none-eabihf \
//!             --example can_tester_g474
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

// ---------------------------------------------------------------------------
// Register-map constants
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

/// FDCAN1 register-block base address (RM0440 §44.4).
const FDCAN1_BASE: usize = 0x4000_6400;

/// Message RAM base address for FDCAN1 (RM0440 §44.3.3).
///
/// On STM32G4 this is a hardware-fixed 212-word (848-byte) SRAM region.
/// Layout is silicon-fixed — do NOT write SIDFC/XIDFC/RXF0C/RXF1C/TXEFC.
const SRAMCAN_BASE: usize = 0x4000_A400;

// ---------------------------------------------------------------------------
// FDCAN1 register offsets (STM32G4-specific)
// ---------------------------------------------------------------------------

const FDCAN_CCCR_OFFSET:  usize = 0x018; // CC control register
const FDCAN_NBTP_OFFSET:  usize = 0x01C; // Nominal bit timing
const FDCAN_RXGFC_OFFSET: usize = 0x080; // RX global filter config
const FDCAN_RXF0S_OFFSET: usize = 0x090; // RX FIFO 0 status
const FDCAN_RXF0A_OFFSET: usize = 0x094; // RX FIFO 0 ack
const FDCAN_TXBC_OFFSET:  usize = 0x0C0; // TX buffer config
const FDCAN_TXFQS_OFFSET: usize = 0x0C4; // TX FIFO/queue status
const FDCAN_TXBAR_OFFSET: usize = 0x0CC; // TX buffer add request

// ---------------------------------------------------------------------------
// FDCAN_CCCR bit masks
// ---------------------------------------------------------------------------

const CCCR_INIT: u32 = 1 << 0;
const CCCR_CCE:  u32 = 1 << 1;
const CCCR_FDOE: u32 = 1 << 8;
const CCCR_BRSE: u32 = 1 << 9;

// ---------------------------------------------------------------------------
// STM32G4 Message RAM layout (hardware-fixed)
// ---------------------------------------------------------------------------

const MRAM_RXF0_OFFSET:  usize = 0x0B0;
const MRAM_TXBUF_OFFSET: usize = 0x278;
const MRAM_ELEMENT_STRIDE: usize = 18 * 4; // 72 bytes per element
const WORD: usize = 4;

const TXBC_VAL: u32 = (3 << 24) | (MRAM_TXBUF_OFFSET as u32 / 4);

// ---------------------------------------------------------------------------
// Bit timing: 500 kbit/s @ 170 MHz PCLK1
//
// NBRP=16, NTSEG1=14 (Tseg1=15 TQ), NTSEG2=3 (Tseg2=4 TQ), NSJW=3.
// Bit = 1 + 15 + 4 = 20 TQ → 170 MHz / (17 × 20) = 500 kbps.
// ---------------------------------------------------------------------------

const NBTP_500KBPS: u32 = {
    let nbrp:   u32 = 16;
    let ntseg1: u32 = 14;
    let ntseg2: u32 =  3;
    let nsjw:   u32 =  3;
    (nsjw << 25) | (nbrp << 16) | (ntseg1 << 8) | ntseg2
};

// ---------------------------------------------------------------------------
// LED pin: PA5 (LD2, NUCLEO-G474RE)
// ---------------------------------------------------------------------------

const LED_PIN: u32 = 5;

// ---------------------------------------------------------------------------
// CAN IDs for ISO-TP (physical addressing, classic CAN 11-bit)
// ---------------------------------------------------------------------------

/// Request CAN ID: tester → server.
const REQUEST_ID: u32 = 0x600;
/// Response CAN ID: server → tester.
const RESPONSE_ID: u32 = 0x601;

// ---------------------------------------------------------------------------
// Number of test cases in the sequence.
// ---------------------------------------------------------------------------

const NUM_TESTS: usize = 5;

// ---------------------------------------------------------------------------
// Raw MMIO helpers
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn reg_read(base: usize, offset: usize) -> u32 {
    unsafe { core::ptr::read_volatile((base + offset) as *const u32) }
}

#[inline(always)]
unsafe fn reg_write(base: usize, offset: usize, val: u32) {
    unsafe { core::ptr::write_volatile((base + offset) as *mut u32, val) }
}

#[inline(always)]
unsafe fn reg_modify(base: usize, offset: usize, mask: u32, bits: u32) {
    let v = unsafe { reg_read(base, offset) };
    unsafe { reg_write(base, offset, (v & !mask) | bits) };
}

#[inline(always)]
unsafe fn mram_read(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
unsafe fn mram_write(addr: usize, val: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

unsafe fn gpio_output(base: usize, pin: u32) {
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,   0b11 << (pin * 2), 0b01 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
    }
}

unsafe fn gpio_set(base: usize, pin: u32, high: bool) {
    let bit = if high { 1 << pin } else { 1 << (pin + 16) };
    unsafe { reg_write(base, GPIO_BSRR_OFFSET, bit) };
}

unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
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
// FDCAN1 initialisation (real bus — no loopback)
// ---------------------------------------------------------------------------

/// Enable FDCAN1 clock, configure PA11 (RX) / PA12 (TX) as AF9.
unsafe fn fdcan1_gpio_clock_init() {
    unsafe {
        reg_modify(RCC_BASE, RCC_AHB2ENR_OFFSET, 0, GPIOAEN);
        let _ = reg_read(RCC_BASE, RCC_AHB2ENR_OFFSET);

        // PA11 = FDCAN1_RX (AF9), PA12 = FDCAN1_TX (AF9).
        gpio_af(GPIOA_BASE, 11, 9);
        gpio_af(GPIOA_BASE, 12, 9);

        // Select PCLK1 as FDCAN kernel clock.
        reg_modify(RCC_BASE, RCC_CCIPR_OFFSET, 0b11 << 24, 0b10 << 24);

        reg_modify(RCC_BASE, RCC_APB1ENR1_OFFSET, 0, FDCAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET);
    }
}

unsafe fn fdcan1_enter_init() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_INIT);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0 {}
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_CCE);
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_FDOE | CCCR_BRSE, 0);
    }
}

unsafe fn fdcan1_leave_init() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_INIT | CCCR_CCE, 0);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) != 0 {}
    }
}

unsafe fn fdcan1_configure_mram() {
    unsafe {
        reg_write(FDCAN1_BASE, FDCAN_TXBC_OFFSET,  TXBC_VAL);
        reg_write(FDCAN1_BASE, FDCAN_RXGFC_OFFSET, 0x0000_0000);
    }
}

/// Full FDCAN1 init for real CAN bus (no loopback).  Returns `true` on success.
unsafe fn fdcan1_init() -> bool {
    unsafe {
        fdcan1_gpio_clock_init();
        fdcan1_enter_init();
        reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);
        fdcan1_configure_mram();
        // NOTE: no loopback — normal bus mode.
        fdcan1_leave_init();
        (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0
    }
}

// ---------------------------------------------------------------------------
// CAN TX / RX helpers
// ---------------------------------------------------------------------------

/// Transmit a standard-ID (11-bit) classic CAN frame.
unsafe fn fdcan1_send(id: u32, data: &[u8]) {
    unsafe {
        loop {
            if (reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET) & (1 << 21)) == 0 {
                break;
            }
        }
        let txfqs   = reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET);
        let put_idx = ((txfqs >> 8) & 0x1F) as usize;
        let txbuf_addr = SRAMCAN_BASE + MRAM_TXBUF_OFFSET + put_idx * MRAM_ELEMENT_STRIDE;

        let t0  = (id & 0x7FF) << 18;
        let dlc = data.len().min(8) as u32;
        let t1  = dlc << 16;

        let mut raw = [0u8; 8];
        let n = data.len().min(8);
        raw[..n].copy_from_slice(&data[..n]);
        let db0 = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let db1 = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]);

        mram_write(txbuf_addr,            t0);
        mram_write(txbuf_addr +     WORD, t1);
        mram_write(txbuf_addr + 2 * WORD, db0);
        mram_write(txbuf_addr + 3 * WORD, db1);

        reg_write(FDCAN1_BASE, FDCAN_TXBAR_OFFSET, 1 << put_idx);
    }
}

/// One received CAN frame from RX FIFO 0.
struct RxFrame {
    id:   u32,
    dlc:  u8,
    data: [u8; 8],
}

/// Read one frame from RX FIFO 0 and acknowledge it.  Returns `None` if empty.
unsafe fn fdcan1_rx_fifo0_read() -> Option<RxFrame> {
    unsafe {
        let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);
        let fill  = rxf0s & 0xF;
        if fill == 0 {
            return None;
        }
        let get_idx  = ((rxf0s >> 8) & 0x3) as usize;
        let elem_addr = SRAMCAN_BASE + MRAM_RXF0_OFFSET + get_idx * MRAM_ELEMENT_STRIDE;

        let r0  = mram_read(elem_addr);
        let r1  = mram_read(elem_addr + WORD);
        let db0 = mram_read(elem_addr + 2 * WORD);
        let db1 = mram_read(elem_addr + 3 * WORD);

        let is_ext = (r0 & (1 << 30)) != 0;
        let id = if is_ext { r0 & 0x1FFF_FFFF } else { (r0 >> 18) & 0x7FF };
        let dlc = ((r1 >> 16) & 0xF) as u8;

        let w0 = db0.to_le_bytes();
        let w1 = db1.to_le_bytes();
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&w0);
        data[4..].copy_from_slice(&w1);

        reg_write(FDCAN1_BASE, FDCAN_RXF0A_OFFSET, get_idx as u32);
        Some(RxFrame { id, dlc, data })
    }
}

// ---------------------------------------------------------------------------
// ISO-TP Single Frame helpers
// ---------------------------------------------------------------------------

/// Encode a UDS request as an ISO-TP Single Frame payload.
fn isotp_encode_sf(uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    let payload_len = uds_data.len().min(7);
    out[0] = payload_len as u8; // PCI: type=0 (SF), length
    out[1..1 + payload_len].copy_from_slice(&uds_data[..payload_len]);
    1 + payload_len
}

/// Decode a received CAN frame as an ISO-TP Single Frame.
///
/// Returns `Some(payload_len)` if valid SF, `None` otherwise.
fn isotp_decode_sf(frame_data: &[u8]) -> Option<usize> {
    let pci = *frame_data.first()?;
    if (pci >> 4) != 0 {
        return None; // not a SF
    }
    let payload_len = (pci & 0x0F) as usize;
    if payload_len == 0 || payload_len > 7 {
        return None;
    }
    Some(payload_len)
}

// ---------------------------------------------------------------------------
// Test helper: send SF request and wait for SF response.
// ---------------------------------------------------------------------------

/// Send a Single Frame UDS request on 0x600 and wait for a Single Frame
/// response on 0x601.
///
/// Returns `Some([u8; 8], dlc)` — the raw CAN frame — on success, or `None`
/// on timeout (1 s).
unsafe fn send_recv_sf(
    req:   &[u8],
    timer: &mut DwtTimer,
) -> Option<([u8; 8], u8)> {
    let mut tx_buf = [0u8; 8];
    let tx_len = isotp_encode_sf(req, &mut tx_buf);
    unsafe { fdcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

    let deadline = timer.system_time_us_64() + 1_000_000; // 1 s timeout
    loop {
        if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
            if f.id == RESPONSE_ID {
                return Some((f.data, f.dlc));
            }
            // Discard frames on other IDs (bus noise, etc.)
        }
        if timer.system_time_us_64() >= deadline {
            return None;
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
    // 2. DWT timer
    // ------------------------------------------------------------------
    let mut timer = DwtTimer::new();
    timer.init(&mut cp.DCB, &mut cp.DWT, sys_clock);

    // ------------------------------------------------------------------
    // 3. UART — USART2 115200 8N1 on PA2 (TX) / PA3 (RX)
    // ------------------------------------------------------------------
    let mut uart = PolledUart::new();
    uart.init();
    let _ = writeln!(uart,
        "\r\n=== CAN tester (real bus) starting @ {} MHz ===",
        sys_clock / 1_000_000);

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    // ------------------------------------------------------------------
    unsafe {
        gpio_output(GPIOA_BASE, LED_PIN);
        gpio_set(GPIOA_BASE, LED_PIN, false);
    }

    // ------------------------------------------------------------------
    // 5. FDCAN1 — PA11/PA12 AF9, 500 kbps, real bus
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Initialising FDCAN1 (real bus, PA11=RX PA12=TX, 500 kbps)...");
    let init_ok = unsafe { fdcan1_init() };
    if !init_ok {
        let _ = writeln!(uart, "FAIL: FDCAN1 did not leave init mode");
        blink_forever(&mut timer, 200_000);
    }
    let _ = writeln!(uart,
        "FDCAN1 ready.  TX=0x{:03X}  Listen=0x{:03X}",
        REQUEST_ID, RESPONSE_ID);

    // ------------------------------------------------------------------
    // 6. Wait 1 second for server to finish booting.
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Waiting 1 s for server to boot...");
    uart.flush();
    let boot_wait = timer.system_time_us_64() + 1_000_000;
    while timer.system_time_us_64() < boot_wait {}

    // ------------------------------------------------------------------
    // 7. Test sequence
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "\r\n--- Test sequence ---");

    let mut pass_count: usize = 0;
    let mut results = [false; NUM_TESTS];

    // ------------------------------------------------------------------
    // Test 1: TesterPresent (0x3E 0x00) → expect SF [02, 7E, 00]
    // ------------------------------------------------------------------
    {
        let req = [0x3E_u8, 0x00];
        let _ = writeln!(uart, "[1] TesterPresent  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect: 0x7E subFunction=0x00
                        let ok = uds.get(0) == Some(&0x7E)
                               && uds.get(1) == Some(&0x00);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[0] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 2: DiagnosticSessionControl extended (0x10 0x03)
    //         → expect SF [07, 50, 03, 00, 19, 01, F4]
    // ------------------------------------------------------------------
    {
        let req = [0x10_u8, 0x03];
        let _ = writeln!(uart, "[2] DiagSessionCtrl extended  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect: 0x50 sessionType=0x03
                        let ok = uds.get(0) == Some(&0x50)
                               && uds.get(1) == Some(&0x03);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[1] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 3: ReadDID unknown DID 0x0001 (0x22 0x00 0x01)
    //         → expect NRC SF [04, 7F, 22, 31]
    // ------------------------------------------------------------------
    {
        let req = [0x22_u8, 0x00, 0x01];
        let _ = writeln!(uart, "[3] ReadDID unknown DID  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect NRC: 0x7F SID=0x22 NRC=0x31 (requestOutOfRange)
                        let ok = uds.get(0) == Some(&0x7F)
                               && uds.get(1) == Some(&0x22)
                               && uds.get(2) == Some(&0x31);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[2] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 4: Unknown service (0xAA)
    //         → expect NRC SF [04, 7F, AA, 11]
    // ------------------------------------------------------------------
    {
        let req = [0xAA_u8];
        let _ = writeln!(uart, "[4] Unknown service 0xAA  req={:02X?}", &req[..]);

        let passed = match unsafe { send_recv_sf(&req, &mut timer) } {
            None => {
                let _ = writeln!(uart, "    FAIL: timeout (no response)");
                false
            }
            Some((data, dlc)) => {
                let dlc = dlc.min(8) as usize;
                match isotp_decode_sf(&data[..dlc]) {
                    None => {
                        let _ = writeln!(uart, "    FAIL: not a SF  raw={:02X?}", &data[..dlc]);
                        false
                    }
                    Some(payload_len) => {
                        let uds = &data[1..1 + payload_len];
                        // Expect NRC: 0x7F SID=0xAA NRC=0x11 (serviceNotSupported)
                        let ok = uds.get(0) == Some(&0x7F)
                               && uds.get(1) == Some(&0xAA)
                               && uds.get(2) == Some(&0x11);
                        let _ = writeln!(uart,
                            "    resp={:02X?}  {}",
                            &data[..dlc],
                            if ok { "PASS" } else { "FAIL: unexpected" });
                        ok
                    }
                }
            }
        };
        results[3] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // Test 5: ReadDID VIN (0x22 0xF1 0x90) → multi-frame response (19 bytes)
    //
    // The server responds with FF (msg_len=19, first 6 bytes) then CFs.
    // We must send FC(CTS) after receiving FF, then collect CFs.
    // ------------------------------------------------------------------
    {
        let req = [0x22_u8, 0xF1, 0x90];
        let _ = writeln!(uart, "[5] ReadDID VIN  req={:02X?}", &req[..]);

        // Send request as SF
        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&req, &mut tx_buf);
        unsafe { fdcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

        // Wait for FF (First Frame) on RESPONSE_ID
        let deadline = timer.system_time_us_64() + 2_000_000; // 2s timeout
        let mut rx_buf = [0u8; 64];
        let mut received: usize = 0;
        let mut msg_len: usize = 0;
        let mut got_ff = false;

        // Phase 1: wait for FF
        loop {
            if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                if f.id == RESPONSE_ID {
                    let pci = f.data[0];
                    let ft = pci >> 4;
                    if ft == 1 {
                        // First Frame: decode msg_len from PCI
                        msg_len = (((pci & 0x0F) as usize) << 8) | (f.data[1] as usize);
                        let first_chunk = (f.dlc.min(8) as usize - 2).min(6).min(msg_len);
                        rx_buf[..first_chunk].copy_from_slice(&f.data[2..2 + first_chunk]);
                        received = first_chunk;
                        got_ff = true;
                        let _ = writeln!(uart, "    FF: msg_len={} first_chunk={}", msg_len, first_chunk);

                        // Send Flow Control: CTS, BS=0, STmin=0
                        let mut fc = [0u8; 8];
                        fc[0] = 0x30; // FC CTS
                        fc[1] = 0x00; // block size = unlimited
                        fc[2] = 0x00; // STmin = 0
                        unsafe { fdcan1_send(REQUEST_ID, &fc) };
                        break;
                    } else if ft == 0 {
                        // SF response (shouldn't happen for VIN but handle it)
                        let plen = (pci & 0x0F) as usize;
                        let _ = writeln!(uart, "    Got SF instead of FF, plen={}", plen);
                        msg_len = plen;
                        rx_buf[..plen.min(7)].copy_from_slice(&f.data[1..1+plen.min(7)]);
                        received = plen.min(7);
                        got_ff = true; // treat as complete
                        break;
                    }
                }
            }
            if timer.system_time_us_64() >= deadline {
                break;
            }
        }

        // Phase 2: collect CFs
        if got_ff && received < msg_len {
            let mut expected_seq: u8 = 1;
            let cf_deadline = timer.system_time_us_64() + 2_000_000;
            while received < msg_len {
                if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                    if f.id == RESPONSE_ID {
                        let pci = f.data[0];
                        if (pci >> 4) == 2 {
                            let seq = pci & 0x0F;
                            if seq != (expected_seq & 0x0F) {
                                let _ = writeln!(uart, "    CF seq mismatch: expected {} got {}", expected_seq & 0x0F, seq);
                                break;
                            }
                            let remaining = msg_len - received;
                            let chunk = remaining.min(7);
                            let dlc = f.dlc.min(8) as usize;
                            let avail = (dlc - 1).min(chunk);
                            rx_buf[received..received + avail].copy_from_slice(&f.data[1..1 + avail]);
                            received += avail;
                            expected_seq = expected_seq.wrapping_add(1);
                            let _ = writeln!(uart, "    CF[{}]: +{} bytes, total={}/{}", seq, avail, received, msg_len);
                        }
                    }
                }
                if timer.system_time_us_64() >= cf_deadline {
                    let _ = writeln!(uart, "    CF timeout: got {}/{} bytes", received, msg_len);
                    break;
                }
            }
        }

        // Verify: expect [62, F1, 90, 'T', 'A', 'K', 'T', 'F', 'L', 'O', 'W', '_', 'G', '4', '7', '4', '_', '0', '1']
        let passed = if !got_ff {
            let _ = writeln!(uart, "    FAIL: timeout (no FF/SF received)");
            false
        } else if received < msg_len {
            let _ = writeln!(uart, "    FAIL: incomplete ({}/{})", received, msg_len);
            false
        } else {
            let ok = received >= 3
                  && rx_buf[0] == 0x62
                  && rx_buf[1] == 0xF1
                  && rx_buf[2] == 0x90
                  && received == 19;
            if ok {
                // Print VIN string
                let vin = &rx_buf[3..received.min(19)];
                let _ = write!(uart, "    VIN=\"");
                for &b in vin {
                    if b >= 0x20 && b < 0x7F {
                        let _ = write!(uart, "{}", b as char);
                    } else {
                        let _ = write!(uart, ".");
                    }
                }
                let _ = writeln!(uart, "\"  PASS");
            } else {
                let _ = writeln!(uart, "    FAIL: unexpected response (len={}, data={:02X?})", received, &rx_buf[..received.min(20)]);
            }
            ok
        };
        results[4] = passed;
        if passed { pass_count += 1; }
    }

    // ------------------------------------------------------------------
    // 8. Summary
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "\r\n=== RESULT: {}/{} PASS ===", pass_count, NUM_TESTS);
    for (i, &ok) in results.iter().enumerate() {
        let _ = writeln!(uart, "  Test {}: {}", i + 1, if ok { "PASS" } else { "FAIL" });
    }
    uart.flush();

    // ------------------------------------------------------------------
    // 9. LED indication and loop
    //
    //    Fast blink (100 ms half-period) = all tests passed.
    //    Slow blink (500 ms half-period) = one or more tests failed.
    // ------------------------------------------------------------------
    let all_pass = pass_count == NUM_TESTS;
    let half_period_us: u64 = if all_pass { 100_000 } else { 500_000 };

    blink_forever(&mut timer, half_period_us);
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

/// Toggle PA5 (LED) indefinitely at the given half-period (µs).  Never returns.
fn blink_forever(timer: &mut DwtTimer, half_period_us: u64) -> ! {
    let mut led_on = false;
    let mut last   = timer.system_time_us_64();
    loop {
        let now = timer.system_time_us_64();
        if now.wrapping_sub(last) >= half_period_us {
            last   = now;
            led_on = !led_on;
            unsafe { gpio_set(GPIOA_BASE, LED_PIN, led_on) };
        }
    }
}
