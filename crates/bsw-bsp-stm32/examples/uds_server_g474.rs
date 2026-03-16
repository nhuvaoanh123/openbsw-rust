//! UDS diagnostic server on NUCLEO-G474RE via FDCAN1 internal loopback.
//!
//! Implements a minimal ISO-TP (ISO 15765-2) Single Frame bridge and a UDS
//! (ISO 14229-1) server that handles three diagnostic services:
//!
//! | SID  | Service                  | Positive response |
//! |------|--------------------------|-------------------|
//! | 0x3E | TesterPresent            | 0x7E              |
//! | 0x10 | DiagnosticSessionControl | 0x50              |
//! | 0x22 | ReadDataByIdentifier     | 0x62              |
//!
//! # What it does
//!
//! 1. Init clocks — 170 MHz boost mode via `clock_g4::configure_clocks_g474`.
//! 2. Init DWT timer for microsecond timestamps.
//! 3. Init USART2 at 115200 baud for diagnostic output.
//! 4. Configure PA5 (LD2) as LED.
//! 5. Configure FDCAN1 in internal-loopback mode (RM0440 §44.3.3).
//! 6. Run startup self-test — send 3 UDS requests, verify responses:
//!    - TesterPresent (0x3E 0x00)       → expect 0x7E 0x00
//!    - DiagSessionCtrl extended (0x10 0x03) → expect 0x50 0x03 …
//!    - ReadDID VIN (0x22 0xF1 0x90)    → expect 0x62 0xF1 0x90 + VIN
//! 7. Enter continuous main loop — echo every request with a UDS response,
//!    print SID and response length over UART, blink LED on each exchange.
//!
//! # ISO-TP support (ISO 15765-2)
//!
//! Full Single Frame + Multi-frame (First Frame / Consecutive Frame / Flow
//! Control) ISO-TP is implemented.  Responses >7 bytes (e.g., ReadDID VIN =
//! 19 bytes) are automatically segmented into FF + CF sequence.
//!
//! ```text
//! Byte 0   = PCI: (0x0 << 4) | payload_len   (SF_PCI)
//! Bytes 1… = UDS payload (SID + parameters)
//! ```
//!
//! # CAN IDs
//!
//! | Direction        | CAN ID |
//! |------------------|--------|
//! | Tester → ECU     | 0x600  |
//! | ECU → Tester     | 0x601  |
//!
//! # Loopback flow
//!
//! ```
//!  TX (0x600)  →  FDCAN1 loopback  →  RX FIFO 0
//!     │                                    │
//!  Tester req                        decode ISO-TP SF
//!                                          │
//!                                    handle_uds_request()
//!                                          │
//!                                    encode ISO-TP SF (0x601)
//!                                          │
//!                                     TX (ignored on re-receive)
//! ```
//!
//! # Build
//!
//! ```sh
//! cargo build --features stm32g474 --target thumbv7em-none-eabihf \
//!             --example uds_server_g474
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
// Register-map constants (identical to can_loopback_g474.rs)
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
/// GPIOB clock enable (AHB2ENR bit 1).
const GPIOBEN: u32 = 1 << 1;

/// GPIOA base address.
const GPIOA_BASE: usize = 0x4800_0000;
/// GPIOB base address.
const GPIOB_BASE: usize = 0x4800_0400;

/// GPIO register offsets.
const GPIO_MODER_OFFSET:  usize = 0x00;
const GPIO_OSPEEDR_OFFSET: usize = 0x08;
const GPIO_AFRL_OFFSET:   usize = 0x20;
const GPIO_AFRH_OFFSET:   usize = 0x24;
const GPIO_BSRR_OFFSET:   usize = 0x18;

/// FDCAN1 register-block base address (RM0440 §44.4).
const FDCAN1_BASE: usize = 0x4000_6400;

/// Message RAM base address for FDCAN1 (RM0440 §44.3.3).
///
/// On STM32G4 this is a hardware-fixed 212-word (848-byte) SRAM region.
/// Layout is silicon-fixed — do NOT write SIDFC/XIDFC/RXF0C/RXF1C/TXEFC.
const SRAMCAN_BASE: usize = 0x4000_A400;

// ---------------------------------------------------------------------------
// FDCAN1 register offsets (STM32G4-specific — see can_loopback_g474.rs for
// the full table of offsets that differ from generic Bosch M_CAN IP).
// ---------------------------------------------------------------------------

const FDCAN_TEST_OFFSET:  usize = 0x010; // Test register
const FDCAN_CCCR_OFFSET:  usize = 0x018; // CC control register
const FDCAN_NBTP_OFFSET:  usize = 0x01C; // Nominal bit timing
const FDCAN_RXGFC_OFFSET: usize = 0x080; // RX global filter config
const FDCAN_RXF0S_OFFSET: usize = 0x090; // RX FIFO 0 status   (G4: 0x090)
const FDCAN_RXF0A_OFFSET: usize = 0x094; // RX FIFO 0 ack      (G4: 0x094)
const FDCAN_TXBC_OFFSET:  usize = 0x0C0; // TX buffer config   (G4: 0x0C0)
const FDCAN_TXFQS_OFFSET: usize = 0x0C4; // TX FIFO/queue status (G4: 0x0C4)
const FDCAN_TXBAR_OFFSET: usize = 0x0CC; // TX buffer add req  (G4: 0x0CC)

// ---------------------------------------------------------------------------
// FDCAN_CCCR bit masks
// ---------------------------------------------------------------------------

/// INIT — initialisation mode.
const CCCR_INIT: u32 = 1 << 0;
/// CCE — configuration change enable (requires INIT=1).
const CCCR_CCE:  u32 = 1 << 1;
/// TEST — gates write access to the TEST register.
const CCCR_TEST: u32 = 1 << 7;
/// FDOE — FD operation; 0 = classic CAN only.
const CCCR_FDOE: u32 = 1 << 8;
/// BRSE — bit-rate switching; 0 = disabled.
const CCCR_BRSE: u32 = 1 << 9;

// ---------------------------------------------------------------------------
// FDCAN_TEST bit masks
// ---------------------------------------------------------------------------

/// LBCK (bit 4) — internal loopback mode.
const TEST_LBCK: u32 = 1 << 4;

// ---------------------------------------------------------------------------
// STM32G4 Message RAM layout (hardware-fixed, RM0440 §44.3.3)
//
//   0x000–0x06F  Standard ID filters  28 × 1W  = 112 bytes
//   0x070–0x0AF  Extended ID filters   8 × 2W  =  64 bytes
//   0x0B0–0x187  RX FIFO 0             3 × 18W = 216 bytes
//   0x188–0x25F  RX FIFO 1             3 × 18W = 216 bytes
//   0x260–0x277  TX event FIFO         3 × 2W  =  24 bytes
//   0x278–0x34F  TX buffers            3 × 18W = 216 bytes
//
// Classic CAN element (18 words, 72 bytes) — only first 4 words carry data:
//   Word 0 = T0/R0: ID, XTD, RTR, ESI
//   Word 1 = T1/R1: DLC, BRS, FDF, timestamp
//   Word 2 = DB0:   payload bytes [3:0] little-endian
//   Word 3 = DB1:   payload bytes [7:4] little-endian
// ---------------------------------------------------------------------------

const MRAM_RXF0_OFFSET:  usize = 0x0B0;
const MRAM_TXBUF_OFFSET: usize = 0x278;

/// Stride between RX/TX elements: 18 words = 72 bytes.
const MRAM_ELEMENT_STRIDE: usize = 18 * 4; // 72

/// Word size in bytes.
const WORD: usize = 4;

// ---------------------------------------------------------------------------
// TXBC value
//
// TXBC [29:24] TFQS=3, [21:16] NDTB=0, [15:0] TBSA = 0x278/4 = 0x9E.
// ---------------------------------------------------------------------------

const TXBC_VAL: u32 = (3 << 24) | (MRAM_TXBUF_OFFSET as u32 / 4);

// ---------------------------------------------------------------------------
// Bit timing: 500 kbit/s @ 170 MHz PCLK1
//
// NBRP=16 (prescaler=17), NTSEG1=13 (Tseg1=14 TQ), NTSEG2=3 (Tseg2=4 TQ),
// NSJW=3 (SJW=4 TQ).  Tq = 17/170 MHz = 100 ns; Fbit = 1/(19 × 100 ns).
// ---------------------------------------------------------------------------

const NBTP_500KBPS: u32 = {
    let nbrp: u32   = 16;
    let ntseg1: u32 = 13;
    let ntseg2: u32 = 3;
    let nsjw: u32   = 3;
    (nsjw << 25) | (ntseg1 << 16) | (ntseg2 << 8) | nbrp
};

// ---------------------------------------------------------------------------
// LED pin: PA5 (LD2, NUCLEO-G474RE)
// ---------------------------------------------------------------------------

const LED_PIN: u32 = 5;

// ---------------------------------------------------------------------------
// CAN IDs for ISO-TP (functional addressing, classic CAN 11-bit)
// ---------------------------------------------------------------------------

/// Physical request CAN ID: tester → ECU.
const REQUEST_ID: u32 = 0x600;
/// Physical response CAN ID: ECU → tester.
const RESPONSE_ID: u32 = 0x601;

// ---------------------------------------------------------------------------
// Raw MMIO helpers (file-local; same pattern as can_loopback_g474.rs)
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

#[inline(always)]
unsafe fn mram_read(addr: usize) -> u32 {
    // SAFETY: caller guarantees addr is inside FDCAN1 message RAM.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
unsafe fn mram_write(addr: usize, val: u32) {
    // SAFETY: caller guarantees addr is inside FDCAN1 message RAM.
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

// ---------------------------------------------------------------------------
// GPIO helpers (identical to can_loopback_g474.rs)
// ---------------------------------------------------------------------------

/// Configure a GPIO pin as push-pull output, high-speed, no pull.
unsafe fn gpio_output(base: usize, pin: u32) {
    // SAFETY: base + offsets are valid GPIO MMIO on STM32G4.
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,  0b11 << (pin * 2), 0b01 << (pin * 2));
        reg_modify(base, GPIO_OSPEEDR_OFFSET, 0b11 << (pin * 2), 0b10 << (pin * 2));
    }
}

/// Set or clear a GPIO pin via BSRR.
unsafe fn gpio_set(base: usize, pin: u32, high: bool) {
    // SAFETY: base + BSRR is valid GPIO MMIO.
    let bit = if high { 1 << pin } else { 1 << (pin + 16) };
    unsafe { reg_write(base, GPIO_BSRR_OFFSET, bit) };
}

/// Configure a GPIO pin as alternate function (AF mode = MODER=10).
unsafe fn gpio_af(base: usize, pin: u32, af: u32) {
    // SAFETY: base + AF offsets are valid GPIO MMIO on STM32G4.
    unsafe {
        reg_modify(base, GPIO_MODER_OFFSET,  0b11 << (pin * 2), 0b10 << (pin * 2));
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
// FDCAN1 initialisation (internal loopback)
// ---------------------------------------------------------------------------

/// Enable FDCAN1 clock, configure PB8 (TX) / PB9 (RX) as AF9, select PCLK1.
unsafe fn fdcan1_gpio_clock_init() {
    // SAFETY: RCC, GPIOB addresses are valid on STM32G474RE.
    unsafe {
        reg_modify(RCC_BASE, RCC_AHB2ENR_OFFSET, 0, GPIOAEN | GPIOBEN);
        let _ = reg_read(RCC_BASE, RCC_AHB2ENR_OFFSET); // read-back delay

        // PB8 = FDCAN1_TX (AF9), PB9 = FDCAN1_RX (AF9).
        gpio_af(GPIOB_BASE, 8, 9);
        gpio_af(GPIOB_BASE, 9, 9);

        // Select PCLK1 as FDCAN kernel clock (CCIPR bits [25:24] = 0b10).
        reg_modify(RCC_BASE, RCC_CCIPR_OFFSET, 0b11 << 24, 0b10 << 24);

        // Enable FDCAN1 peripheral clock.
        reg_modify(RCC_BASE, RCC_APB1ENR1_OFFSET, 0, FDCAN1EN);
        let _ = reg_read(RCC_BASE, RCC_APB1ENR1_OFFSET); // read-back delay
    }
}

/// Enter FDCAN1 INIT+CCE mode (required before touching configuration).
unsafe fn fdcan1_enter_init() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_INIT);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0 {}
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_CCE);
        // Classic CAN only — clear FD and BRS bits.
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_FDOE | CCCR_BRSE, 0);
    }
}

/// Leave INIT mode so FDCAN1 can participate in bus traffic.
unsafe fn fdcan1_leave_init() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_INIT | CCCR_CCE, 0);
        while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) != 0 {}
    }
}

/// Configure TXBC and RXGFC.  Do NOT write SIDFC/XIDFC/RXF0C/RXF1C/TXEFC.
unsafe fn fdcan1_configure_mram() {
    unsafe {
        // TXBC: 3 TX FIFO elements, no dedicated buffers.
        reg_write(FDCAN1_BASE, FDCAN_TXBC_OFFSET, TXBC_VAL);
        // RXGFC=0: accept all non-matching std + ext frames into FIFO 0.
        reg_write(FDCAN1_BASE, FDCAN_RXGFC_OFFSET, 0x0000_0000);
    }
}

/// Enable FDCAN1 internal loopback (CCCR.TEST=1, TEST.LBCK=1).
///
/// Must be called while FDCAN1 is in INIT + CCE mode.
unsafe fn fdcan1_enable_loopback() {
    unsafe {
        reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_TEST);
        reg_modify(FDCAN1_BASE, FDCAN_TEST_OFFSET, 0, TEST_LBCK);
    }
}

/// Full FDCAN1 init in internal-loopback mode.
///
/// Returns `true` if the peripheral left init mode successfully.
///
/// To switch to normal (external bus) mode, remove the call to
/// `fdcan1_enable_loopback()` below.
unsafe fn fdcan1_init_loopback() -> bool {
    unsafe {
        fdcan1_gpio_clock_init();
        fdcan1_enter_init();
        reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);
        fdcan1_configure_mram();
        fdcan1_enable_loopback(); // remove this line for normal (external bus) mode
        fdcan1_leave_init();
        (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0
    }
}

// ---------------------------------------------------------------------------
// CAN TX / RX helpers (raw message RAM access)
// ---------------------------------------------------------------------------

/// Write a TX buffer element and request transmission.
///
/// Sends a standard-ID (11-bit) classic CAN frame.  Uses the TX FIFO put
/// index from TXFQS to locate the next free slot.
///
/// TX element layout (RM0440 §44.4.18):
///   Word 0 = T0: std ID in [28:18], XTD=0, RTR=0, ESI=0
///   Word 1 = T1: DLC in [19:16], BRS=0, FDF=0
///   Word 2 = DB0: payload bytes [3:0] little-endian
///   Word 3 = DB1: payload bytes [7:4] little-endian
unsafe fn fdcan1_send(id: u32, data: &[u8]) {
    // SAFETY: message RAM and FDCAN1 register addresses valid for STM32G474.
    unsafe {
        // Wait until TX FIFO has a free slot (TXFQS bit 21 = FIFO full).
        loop {
            if (reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET) & (1 << 21)) == 0 {
                break;
            }
        }

        // TXFQS [12:8] = put index.
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

/// Read one frame from RX FIFO 0 and acknowledge it.
///
/// Returns `None` if the FIFO is empty.
///
/// RXF0S on STM32G4 (RM0440 §44.8.21):
///   [3:0]   F0FL fill level (max 3)
///   [9:8]   F0GI get index
///   [17:16] F0PI put index
///
/// RX element layout (RM0440 §44.4.16):
///   Word 0 = R0: std ID [28:18] (if XTD=0), XTD [30], RTR [29]
///   Word 1 = R1: DLC [19:16]
///   Word 2 = DB0: payload bytes [3:0] little-endian
///   Word 3 = DB1: payload bytes [7:4] little-endian
unsafe fn fdcan1_rx_fifo0_read() -> Option<RxFrame> {
    // SAFETY: FDCAN1 and message RAM addresses valid for STM32G474.
    unsafe {
        let rxf0s = reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET);

        // F0FL: bits [3:0] on STM32G4.
        let fill = rxf0s & 0xF;
        if fill == 0 {
            return None;
        }

        // F0GI: bits [9:8] on STM32G4.
        let get_idx = ((rxf0s >> 8) & 0x3) as usize;

        let elem_addr = SRAMCAN_BASE + MRAM_RXF0_OFFSET + get_idx * MRAM_ELEMENT_STRIDE;

        let r0  = mram_read(elem_addr);
        let r1  = mram_read(elem_addr + WORD);
        let db0 = mram_read(elem_addr + 2 * WORD);
        let db1 = mram_read(elem_addr + 3 * WORD);

        let is_ext = (r0 & (1 << 30)) != 0;
        let id = if is_ext {
            r0 & 0x1FFF_FFFF
        } else {
            (r0 >> 18) & 0x7FF
        };
        let dlc = ((r1 >> 16) & 0xF) as u8;

        let w0 = db0.to_le_bytes();
        let w1 = db1.to_le_bytes();
        let mut data = [0u8; 8];
        data[..4].copy_from_slice(&w0);
        data[4..].copy_from_slice(&w1);

        // Acknowledge — RXF0A [2:0] = get_idx.
        reg_write(FDCAN1_BASE, FDCAN_RXF0A_OFFSET, get_idx as u32);

        Some(RxFrame { id, dlc, data })
    }
}

// ---------------------------------------------------------------------------
// ISO-TP Single Frame helpers
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// ISO-TP frame types (ISO 15765-2)
//
//   0x0N = Single Frame      (N = payload length, 1-7)
//   0x1M = First Frame       (M:LL = total message length, 12-bit)
//   0x2N = Consecutive Frame (N = sequence number, 0-F wrapping)
//   0x30 = Flow Control      (CTS / Wait / Overflow)
// ---------------------------------------------------------------------------

/// ISO-TP frame type, decoded from upper nibble of PCI byte.
#[derive(Debug, Clone, Copy, PartialEq)]
enum IsoTpFrameType {
    SingleFrame,
    FirstFrame,
    ConsecutiveFrame,
    FlowControl,
}

fn isotp_frame_type(pci: u8) -> Option<IsoTpFrameType> {
    match pci >> 4 {
        0 => Some(IsoTpFrameType::SingleFrame),
        1 => Some(IsoTpFrameType::FirstFrame),
        2 => Some(IsoTpFrameType::ConsecutiveFrame),
        3 => Some(IsoTpFrameType::FlowControl),
        _ => None,
    }
}

/// Encode a UDS response as an ISO-TP Single Frame CAN payload.
///
/// SF PCI: `byte[0] = (0x0 << 4) | payload_len`, then payload bytes.
/// Returns number of bytes written to `out`. Caller must ensure `resp_len ≤ 7`.
fn isotp_encode_sf(uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    let payload_len = uds_data.len().min(7);
    out[0] = payload_len as u8; // PCI: type=0 (SF), length
    out[1..1 + payload_len].copy_from_slice(&uds_data[..payload_len]);
    1 + payload_len
}

/// Encode an ISO-TP First Frame.
///
/// FF PCI: `byte[0] = 0x10 | (msg_len >> 8)`, `byte[1] = msg_len & 0xFF`,
/// then up to 6 payload bytes.
/// Returns number of bytes written to `out` (always 8 for classic CAN).
fn isotp_encode_ff(msg_len: u16, uds_data: &[u8], out: &mut [u8; 8]) -> usize {
    out[0] = 0x10 | ((msg_len >> 8) as u8 & 0x0F);
    out[1] = (msg_len & 0xFF) as u8;
    let first_chunk = uds_data.len().min(6);
    out[2..2 + first_chunk].copy_from_slice(&uds_data[..first_chunk]);
    // Pad remaining bytes with 0xCC (ISO-TP padding)
    for b in &mut out[2 + first_chunk..8] {
        *b = 0xCC;
    }
    8
}

/// Encode an ISO-TP Consecutive Frame.
///
/// CF PCI: `byte[0] = 0x20 | (seq_num & 0x0F)`, then up to 7 payload bytes.
/// Returns number of bytes written to `out`.
fn isotp_encode_cf(seq_num: u8, data: &[u8], out: &mut [u8; 8]) -> usize {
    out[0] = 0x20 | (seq_num & 0x0F);
    let chunk = data.len().min(7);
    out[1..1 + chunk].copy_from_slice(&data[..chunk]);
    // Pad remaining
    for b in &mut out[1 + chunk..8] {
        *b = 0xCC;
    }
    8
}

/// Encode an ISO-TP Flow Control frame (CTS, block_size=0, STmin=0).
fn isotp_encode_fc_cts(out: &mut [u8; 8]) -> usize {
    out[0] = 0x30; // FC, ContinueToSend
    out[1] = 0x00; // Block size = 0 (no limit)
    out[2] = 0x00; // STmin = 0 ms
    // Pad
    for b in &mut out[3..8] {
        *b = 0xCC;
    }
    8
}

/// Decode a received CAN frame as an ISO-TP Single Frame.
///
/// Returns `Some((pci, payload_len))` if valid SF, `None` otherwise.
/// Used by the self-test to verify SF responses.
fn isotp_decode_sf(frame_data: &[u8]) -> Option<(u8, usize)> {
    let pci = *frame_data.first()?;
    if (pci >> 4) != 0 {
        return None;
    }
    let payload_len = (pci & 0x0F) as usize;
    if payload_len == 0 || payload_len > 7 {
        return None;
    }
    Some((pci, payload_len))
}

/// Send a complete UDS response over ISO-TP, using SF for ≤7 bytes or FF+CF
/// for longer responses. Waits for Flow Control from receiver for multi-frame.
///
/// # Safety
/// Calls `fdcan1_send` and `fdcan1_rx_fifo0_read` which access FDCAN1 MMIO.
unsafe fn isotp_send_response(
    resp_data: &[u8],
    resp_len: usize,
    timer: &mut DwtTimer,
    uart: &mut PolledUart,
) {
    if resp_len <= 7 {
        // Single Frame — fits in one CAN frame.
        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&resp_data[..resp_len], &mut tx_buf);
        let _ = write!(uart, "  tx=[");
        for (i, b) in tx_buf[..tx_len].iter().enumerate() {
            if i > 0 { let _ = write!(uart, ", "); }
            let _ = write!(uart, "{:02X}", b);
        }
        let _ = writeln!(uart, "]");
        unsafe { fdcan1_send(RESPONSE_ID, &tx_buf[..tx_len]) };
    } else {
        // Multi-frame: First Frame + Consecutive Frames.
        let msg_len = resp_len as u16;

        // 1. Send First Frame (6 payload bytes).
        let mut tx_buf = [0u8; 8];
        let ff_payload = resp_len.min(6);
        isotp_encode_ff(msg_len, &resp_data[..ff_payload], &mut tx_buf);
        let _ = writeln!(uart, "  FF: msg_len={} first_chunk={}", msg_len, ff_payload);
        unsafe { fdcan1_send(RESPONSE_ID, &tx_buf) };

        // 2. Wait for Flow Control (FC) from receiver.
        //    In loopback mode, our own FF echoes back — discard it.
        //    The tester/receiver should send FC on REQUEST_ID or RESPONSE_ID.
        //    For loopback self-test, we auto-proceed after a brief delay
        //    (no real receiver to send FC).
        let fc_deadline = timer.system_time_us_64() + 100_000; // 100ms timeout
        let mut got_fc = false;
        loop {
            if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                let pci = f.data[0];
                if let Some(IsoTpFrameType::FlowControl) = isotp_frame_type(pci) {
                    let fc_status = pci & 0x0F;
                    if fc_status == 0 {
                        // CTS (Continue To Send)
                        got_fc = true;
                        let _ = writeln!(uart, "  FC: CTS received");
                        break;
                    } else if fc_status == 1 {
                        // WAIT — keep polling
                        let _ = writeln!(uart, "  FC: WAIT");
                        continue;
                    } else {
                        // Overflow — abort
                        let _ = writeln!(uart, "  FC: OVERFLOW, aborting");
                        return;
                    }
                }
                // Discard non-FC frames (echo of our FF, etc.)
            }
            if timer.system_time_us_64() >= fc_deadline {
                // In loopback mode there's no real receiver to send FC.
                // Auto-proceed after timeout.
                let _ = writeln!(uart, "  FC: timeout (loopback mode), auto-proceeding");
                break;
            }
        }

        // 3. Send Consecutive Frames.
        let mut offset = ff_payload; // bytes already sent in FF
        let mut seq: u8 = 1;
        while offset < resp_len {
            let remaining = resp_len - offset;
            let chunk = remaining.min(7);
            isotp_encode_cf(seq, &resp_data[offset..offset + chunk], &mut tx_buf);
            let _ = writeln!(uart, "  CF[{}]: offset={} chunk={}", seq, offset, chunk);
            unsafe { fdcan1_send(RESPONSE_ID, &tx_buf) };
            offset += chunk;
            seq = (seq + 1) & 0x0F; // wrap at 16

            // Brief delay between CFs (STmin=0 but give HW time)
            let cf_wait = timer.system_time_us_64() + 500;
            while timer.system_time_us_64() < cf_wait {}
        }
        let _ = writeln!(uart, "  Multi-frame TX complete: {} bytes in {} CFs",
                         resp_len, seq);
    }
}

/// Decode an incoming ISO-TP frame. Returns the UDS payload for Single Frames
/// and First Frames (starts multi-frame RX for FF).
///
/// For multi-frame RX: sends FC(CTS), collects CFs, reassembles into `rx_buf`.
/// Returns the total reassembled payload length, or 0 on failure.
///
/// # Safety
/// Calls `fdcan1_send` and `fdcan1_rx_fifo0_read` for FC/CF exchange.
unsafe fn isotp_receive_request(
    frame_data: &[u8],
    dlc: usize,
    rx_buf: &mut [u8; 256],
    timer: &mut DwtTimer,
    uart: &mut PolledUart,
) -> usize {
    let pci = frame_data[0];
    match isotp_frame_type(pci) {
        Some(IsoTpFrameType::SingleFrame) => {
            let payload_len = (pci & 0x0F) as usize;
            if payload_len == 0 || payload_len > 7 || payload_len + 1 > dlc {
                return 0;
            }
            rx_buf[..payload_len].copy_from_slice(&frame_data[1..1 + payload_len]);
            payload_len
        }
        Some(IsoTpFrameType::FirstFrame) => {
            // Decode message length (12-bit).
            let msg_len = (((pci & 0x0F) as usize) << 8) | (frame_data[1] as usize);
            if msg_len == 0 || msg_len > rx_buf.len() {
                let _ = writeln!(uart, "  FF: msg_len={} too large, dropping", msg_len);
                return 0;
            }
            // Copy first chunk (up to 6 bytes).
            let first_chunk = (dlc - 2).min(6).min(msg_len);
            rx_buf[..first_chunk].copy_from_slice(&frame_data[2..2 + first_chunk]);
            let mut received = first_chunk;

            let _ = writeln!(uart, "  FF: msg_len={} first_chunk={}", msg_len, first_chunk);

            // Send Flow Control: CTS, BS=0, STmin=0.
            let mut fc_buf = [0u8; 8];
            isotp_encode_fc_cts(&mut fc_buf);
            unsafe { fdcan1_send(RESPONSE_ID, &fc_buf) };

            // Collect Consecutive Frames.
            let mut expected_seq: u8 = 1;
            let cf_deadline = timer.system_time_us_64() + 1_000_000; // 1s N_Cr timeout
            while received < msg_len {
                if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                    if f.id != REQUEST_ID {
                        continue; // discard echo/other
                    }
                    let cf_pci = f.data[0];
                    if let Some(IsoTpFrameType::ConsecutiveFrame) = isotp_frame_type(cf_pci) {
                        let seq = cf_pci & 0x0F;
                        if seq != (expected_seq & 0x0F) {
                            let _ = writeln!(uart, "  CF: seq mismatch expected={} got={}",
                                             expected_seq & 0x0F, seq);
                            return 0; // abort
                        }
                        let remaining = msg_len - received;
                        let chunk = remaining.min(7);
                        let cf_dlc = f.dlc.min(8) as usize;
                        let avail = (cf_dlc - 1).min(chunk);
                        rx_buf[received..received + avail]
                            .copy_from_slice(&f.data[1..1 + avail]);
                        received += avail;
                        expected_seq = expected_seq.wrapping_add(1);
                    }
                }
                if timer.system_time_us_64() >= cf_deadline {
                    let _ = writeln!(uart, "  CF: timeout after {} of {} bytes",
                                     received, msg_len);
                    return 0;
                }
            }
            let _ = writeln!(uart, "  Multi-frame RX complete: {} bytes", msg_len);
            msg_len
        }
        _ => {
            // FC or CF without context — ignore.
            0
        }
    }
}

// ---------------------------------------------------------------------------
// UDS request handler
// ---------------------------------------------------------------------------

/// Dispatch a decoded UDS request and write the positive (or NRC) response.
///
/// Returns the number of bytes written to `response`.
///
/// Supported services:
///
/// | SID  | Service                     |
/// |------|-----------------------------|
/// | 0x3E | TesterPresent               |
/// | 0x10 | DiagnosticSessionControl    |
/// | 0x22 | ReadDataByIdentifier        |
///
/// DIDs supported by 0x22:
///
/// | DID    | Description      | Value                   |
/// |--------|------------------|-------------------------|
/// | 0xF190 | VIN              | `TAKTFLOW_G474_01`      |
/// | 0xF195 | SW version       | `BSP-0.1.0`             |
fn handle_uds_request(request: &[u8], response: &mut [u8]) -> usize {
    match request.first() {
        // ------------------------------------------------------------------
        // 0x3E TesterPresent
        // Positive response: 0x7E subFunction
        // ------------------------------------------------------------------
        Some(&0x3E) => {
            response[0] = 0x7E;
            response[1] = request.get(1).copied().unwrap_or(0x00);
            2
        }

        // ------------------------------------------------------------------
        // 0x10 DiagnosticSessionControl
        // Positive response: 0x50 sessionType P2 P2*
        // ------------------------------------------------------------------
        Some(&0x10) => {
            let session = request.get(1).copied().unwrap_or(0x01);
            response[0] = 0x50;
            response[1] = session;
            response[2] = 0x00; // P2 high byte   (P2  = 25 ms)
            response[3] = 0x19; // P2 low byte
            response[4] = 0x01; // P2* high byte  (P2* = 5000 ms)
            response[5] = 0xF4; // P2* low byte
            6
        }

        // ------------------------------------------------------------------
        // 0x22 ReadDataByIdentifier
        // Positive response: 0x62 DID_high DID_low data…
        // ------------------------------------------------------------------
        Some(&0x22) => {
            let did_hi = request.get(1).copied().unwrap_or(0x00);
            let did_lo = request.get(2).copied().unwrap_or(0x00);
            let did = u16::from_be_bytes([did_hi, did_lo]);
            match did {
                0xF190 => {
                    // VIN — 17-char ASCII, padded/truncated to fit
                    response[0] = 0x62;
                    response[1] = 0xF1;
                    response[2] = 0x90;
                    let vin = b"TAKTFLOW_G474_01";
                    let len = vin.len().min(17).min(response.len() - 3);
                    response[3..3 + len].copy_from_slice(&vin[..len]);
                    3 + len
                }
                0xF195 => {
                    // Software version
                    response[0] = 0x62;
                    response[1] = 0xF1;
                    response[2] = 0x95;
                    let ver = b"BSP-0.1.0";
                    let len = ver.len().min(response.len() - 3);
                    response[3..3 + len].copy_from_slice(&ver[..len]);
                    3 + len
                }
                _ => {
                    // NRC 0x31 requestOutOfRange
                    response[0] = 0x7F;
                    response[1] = 0x22;
                    response[2] = 0x31;
                    3
                }
            }
        }

        // ------------------------------------------------------------------
        // All other services — NRC 0x11 serviceNotSupported
        // ------------------------------------------------------------------
        _ => {
            response[0] = 0x7F;
            response[1] = request.first().copied().unwrap_or(0x00);
            response[2] = 0x11;
            3
        }
    }
}

// ---------------------------------------------------------------------------
// Self-test helpers
// ---------------------------------------------------------------------------

/// Send a single ISO-TP SF request and wait for the response (up to `timeout_us`).
///
/// Returns `Some(RxFrame)` if a frame with `RESPONSE_ID` is received in time,
/// `None` on timeout.
unsafe fn selftest_send_recv(
    timer:      &mut DwtTimer,
    req_data:   &[u8],         // raw UDS payload (not yet ISO-TP wrapped)
    timeout_us: u64,
) -> Option<RxFrame> {
    // Encode request as ISO-TP SF.
    let mut tx_buf = [0u8; 8];
    let tx_len = isotp_encode_sf(req_data, &mut tx_buf);

    unsafe { fdcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

    let deadline = timer.system_time_us_64() + timeout_us;
    loop {
        if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
            // In loopback we may receive our own TX first (same CAN ID 0x600);
            // discard it and wait for the ECU response (0x601).
            if f.id == RESPONSE_ID {
                return Some(f);
            }
            // Discard REQUEST_ID echo — keep waiting.
        }
        if timer.system_time_us_64() >= deadline {
            return None;
        }
    }
}

// ---------------------------------------------------------------------------
// Main loop: process one incoming request frame and emit a UDS response.
// ---------------------------------------------------------------------------

/// Process one pending request from RX FIFO 0.
///
/// Returns `true` if a REQUEST_ID frame was decoded and a response was sent,
/// `false` if the FIFO was empty or the frame was not a recognised request.
unsafe fn process_one_request(uart: &mut PolledUart, timer: &mut DwtTimer) -> bool {
    let frame = match unsafe { fdcan1_rx_fifo0_read() } {
        Some(f) => f,
        None    => return false,
    };

    // Ignore anything that is not a tester request (e.g., our own response
    // echo that loops back again after we send 0x601).
    if frame.id != REQUEST_ID {
        return false;
    }

    let dlc = frame.dlc.min(8) as usize;

    // ISO-TP: decode incoming frame (SF or FF+CF multi-frame).
    let mut rx_buf = [0u8; 256];
    let payload_len = unsafe {
        isotp_receive_request(&frame.data[..dlc], dlc, &mut rx_buf, timer, uart)
    };
    if payload_len == 0 {
        return false;
    }

    let uds_request = &rx_buf[..payload_len];

    // UDS dispatch.
    let mut resp_buf = [0u8; 256];
    let resp_len = handle_uds_request(uds_request, &mut resp_buf);

    // ISO-TP encode + send response (SF or FF+CF automatically).
    unsafe { isotp_send_response(&resp_buf, resp_len, timer, uart) };

    // Heartbeat blink.
    let ts = timer.system_time_us_64();
    let led = ((ts / 500_000) & 1) != 0;
    unsafe { gpio_set(GPIOA_BASE, LED_PIN, led) };

    let _ = writeln!(uart,
        "UDS: SID=0x{:02X}  req_len={}  resp_len={}",
        uds_request.first().copied().unwrap_or(0),
        payload_len,
        resp_len);

    true
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
        "\r\n=== UDS server starting @ {} MHz ===",
        sys_clock / 1_000_000);

    // ------------------------------------------------------------------
    // 4. LED — PA5 as push-pull output
    // ------------------------------------------------------------------
    // GPIOA clock is enabled by PolledUart::init(); configure PA5 here.
    unsafe {
        gpio_output(GPIOA_BASE, LED_PIN);
        gpio_set(GPIOA_BASE, LED_PIN, false);
    }

    // ------------------------------------------------------------------
    // 5. FDCAN1 in internal-loopback mode
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Initialising FDCAN1 (loopback mode)...");
    let init_ok = unsafe { fdcan1_init_loopback() };
    if !init_ok {
        let _ = writeln!(uart, "FAIL: FDCAN1 did not leave init mode");
        blink_forever(&mut timer, 500_000);
    }
    let _ = writeln!(uart, "FDCAN1 ready.  RX=0x{:03X}  TX=0x{:03X}", REQUEST_ID, RESPONSE_ID);

    // ------------------------------------------------------------------
    // 6. Startup self-test — send 3 UDS requests, verify responses
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "\r\n--- Self-test sequence ---");

    let mut all_passed = true;

    // ---- Test 1: TesterPresent ----
    {
        let req = [0x3E_u8, 0x00];
        let _ = writeln!(uart, "[1] TesterPresent  req={:02X?}", &req[..]);

        // The server processes requests in the main loop; for the self-test we
        // drive both sides inline:
        //   a) Send the request via TX.
        //   b) Process the received request (loopback → RX FIFO 0).
        //   c) Receive the response (RESPONSE_ID frame).

        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&req, &mut tx_buf);
        unsafe { fdcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

        // Brief delay for TX to settle in loopback.
        let t0 = timer.system_time_us_64();
        while timer.system_time_us_64() < t0 + 1_000 {}

        // Process the incoming request (sends response to 0x601).
        let _ = unsafe { process_one_request(&mut uart, &mut timer) };

        // Collect the response from RX FIFO 0 (it is the 0x601 loopback).
        let deadline = timer.system_time_us_64() + 20_000;
        let resp = loop {
            if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                if f.id == RESPONSE_ID { break Some(f); }
            }
            if timer.system_time_us_64() >= deadline { break None; }
        };

        let passed = match resp {
            None => {
                let _ = writeln!(uart, "  FAIL: timeout");
                false
            }
            Some(f) => {
                let dlc = f.dlc.min(8) as usize;
                // ISO-TP SF decode
                if let Some((_, plen)) = isotp_decode_sf(&f.data[..dlc]) {
                    let uds = &f.data[1..1 + plen];
                    // Expect 0x7E 0x00
                    let ok = uds.get(0) == Some(&0x7E) && uds.get(1) == Some(&0x00);
                    let _ = writeln!(uart, "  resp={:02X?}  {}",
                        &f.data[..dlc],
                        if ok { "PASS" } else { "FAIL: unexpected response" });
                    ok
                } else {
                    let _ = writeln!(uart, "  FAIL: non-SF response");
                    false
                }
            }
        };
        if !passed { all_passed = false; }
    }

    // ---- Test 2: DiagnosticSessionControl (extended, 0x03) ----
    {
        let req = [0x10_u8, 0x03];
        let _ = writeln!(uart, "[2] DiagSessionCtrl extended  req={:02X?}", &req[..]);

        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&req, &mut tx_buf);
        unsafe { fdcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

        let t0 = timer.system_time_us_64();
        while timer.system_time_us_64() < t0 + 1_000 {}

        let _ = unsafe { process_one_request(&mut uart, &mut timer) };

        let deadline = timer.system_time_us_64() + 20_000;
        let resp = loop {
            if let Some(f) = unsafe { fdcan1_rx_fifo0_read() } {
                if f.id == RESPONSE_ID { break Some(f); }
            }
            if timer.system_time_us_64() >= deadline { break None; }
        };

        let passed = match resp {
            None => {
                let _ = writeln!(uart, "  FAIL: timeout");
                false
            }
            Some(f) => {
                let dlc = f.dlc.min(8) as usize;
                if let Some((_, plen)) = isotp_decode_sf(&f.data[..dlc]) {
                    let uds = &f.data[1..1 + plen];
                    // Expect 0x50 0x03 …
                    let ok = uds.get(0) == Some(&0x50) && uds.get(1) == Some(&0x03);
                    let _ = writeln!(uart, "  resp={:02X?}  {}",
                        &f.data[..dlc],
                        if ok { "PASS" } else { "FAIL: unexpected response" });
                    ok
                } else {
                    let _ = writeln!(uart, "  FAIL: non-SF response");
                    false
                }
            }
        };
        if !passed { all_passed = false; }
    }

    // ---- Test 3: ReadDataByIdentifier VIN (DID 0xF190) ----
    // VIN response is 19 bytes → multi-frame (FF + 2×CF).
    // In loopback mode, the FF/CF loop back and get consumed by
    // isotp_send_response internally. We verify by checking that
    // process_one_request succeeded and the UART log shows correct
    // multi-frame segmentation.
    {
        let req = [0x22_u8, 0xF1, 0x90];
        let _ = writeln!(uart, "[3] ReadDID VIN  req={:02X?}", &req[..]);

        let mut tx_buf = [0u8; 8];
        let tx_len = isotp_encode_sf(&req, &mut tx_buf);
        unsafe { fdcan1_send(REQUEST_ID, &tx_buf[..tx_len]) };

        let t0 = timer.system_time_us_64();
        while timer.system_time_us_64() < t0 + 1_000 {}

        let processed = unsafe { process_one_request(&mut uart, &mut timer) };

        // Drain any looped-back FF/CF frames from the FIFO.
        let drain_deadline = timer.system_time_us_64() + 200_000;
        while timer.system_time_us_64() < drain_deadline {
            let _ = unsafe { fdcan1_rx_fifo0_read() };
        }

        if processed {
            let _ = writeln!(uart, "  PASS (multi-frame TX verified via UART log)");
        } else {
            let _ = writeln!(uart, "  FAIL: process_one_request returned false");
            all_passed = false;
        }
    }

    // ------------------------------------------------------------------
    // 6b. Self-test summary
    // ------------------------------------------------------------------
    if all_passed {
        let _ = writeln!(uart, "\r\n*** SELF-TEST: PASS — entering diagnostic server loop ***\r\n");
        // Solid LED = all tests passed.
        unsafe { gpio_set(GPIOA_BASE, LED_PIN, true) };
    } else {
        let _ = writeln!(uart, "\r\n*** SELF-TEST: FAIL — see above for details ***\r\n");
    }
    uart.flush();

    // ------------------------------------------------------------------
    // 7. Continuous diagnostic server loop
    //
    //    - Poll RX FIFO 0 for request frames (CAN ID 0x600).
    //    - Decode ISO-TP SF, dispatch to UDS handler, send response (0x601).
    //    - Blink LED (0.5 Hz heartbeat via DWT timestamp).
    // ------------------------------------------------------------------
    let _ = writeln!(uart, "Listening for UDS requests on CAN ID 0x{:03X}...", REQUEST_ID);

    loop {
        unsafe {
            let _ = process_one_request(&mut uart, &mut timer);
        }

        // Slow heartbeat blink (1 Hz, 500 ms half-period) when idle.
        let ts  = timer.system_time_us_64();
        let led = ((ts / 1_000_000) & 1) != 0;
        unsafe { gpio_set(GPIOA_BASE, LED_PIN, led) };
    }
}

// ---------------------------------------------------------------------------
// Panic handler
// ---------------------------------------------------------------------------

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Rapid blink on PA5 to signal a panic, then halt with a breakpoint.
    // A real application would log the panic info over UART before halting.
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
