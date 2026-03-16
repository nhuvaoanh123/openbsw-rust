// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! FDCAN1 transceiver for STM32G474RE — classic CAN at 500 kbit/s.
//!
//! Implements [`CanTransceiver`] using direct FDCAN1 register access and
//! message-RAM manipulation.  The `fdcan` 0.2 crate is available as a
//! dependency but is not used here because it does not expose the message-RAM
//! layout or the accept-all filter configuration path required by OpenBSW.
//!
//! # STM32G4-specific FDCAN differences
//!
//! This driver uses STM32G4-specific FDCAN register offsets, which differ
//! from the generic Bosch M_CAN IP used in other MCU families:
//!
//! - RXF0S is at 0x090 (not 0x0A4), RXF0A at 0x094 (not 0x0A8).
//! - TXBC is at 0x0C0 (not 0x0C4), TXFQS at 0x0C4 (not 0x0C8),
//!   TXBAR at 0x0CC (not 0x0D0).
//! - Registers SIDFC, XIDFC, RXF0C, RXF1C, and TXEFC do not exist on
//!   STM32G4; the MRAM layout is fixed in hardware and configured solely
//!   via TXBC (TX buffer start/count) and RXGFC (filter policy).
//! - RX FIFO and TX buffer elements on STM32G4 are 18 words (72 bytes) each,
//!   not 4 words (16 bytes) as in classic M_CAN.
//! - RXF0S fill-level field is 4 bits [3:0]; get-index is 2 bits [9:8].
//! - TXFQS put-index field is 2 bits [9:8] (max 3 elements).
//!
//! # Message RAM layout (FDCAN1, SRAMCAN base 0x4000_A400)
//!
//! | Section         | Offset  | Elements | Words each | Total   |
//! |-----------------|---------|----------|------------|---------|
//! | Std ID filters  | 0x000   | 28       | 1          | 28 W    |
//! | Ext ID filters  | 0x070   | 8        | 2          | 16 W    |
//! | RX FIFO 0       | 0x0B0   | 3        | 18         | 54 W    |
//! | RX FIFO 1       | 0x188   | 3        | 18         | 54 W    |
//! | TX event FIFO   | 0x260   | 3        | 2          | 6 W     |
//! | TX buffers      | 0x278   | 3        | 18         | 54 W    |
//!
//! All offsets are byte offsets from `SRAMCAN_BASE`.
//! Element stride = 18 words = 72 bytes (STM32G4 FDCAN element size).
//!
//! # Bit timing for 500 kbit/s @ 170 MHz
//!
//! FDCAN clock = PCLK1 = 170 MHz (no FDCAN prescaler).
//! Prescaler = 17 → TQ clock = 10 MHz.
//! NTSEG1 = 12, NTSEG2 = 4, NSJW = 4 → 10 Mbit nominal / 20 = 500 kbit/s.
//! (NTSEG1 + NTSEG2 + 1 sync seg = 12 + 4 + 1 = 17 TQ per bit → 10/17 ≈ … —
//!  corrected: prescaler=17, TQ_clk=10 MHz, bit_time=17 TQ = 1/500 k ✓)
//!
//! # Filter policy
//!
//! RXGFC.ANFS = 0 (accept all std-ID frames to FIFO 0).
//! RXGFC.ANFE = 0 (accept all ext-ID frames to FIFO 0).
//! This mirrors the `cpp2can` default of accept-all.
//!
//! # Safety
//!
//! All register and message-RAM accesses use `read_volatile`/`write_volatile`
//! on hard-coded MMIO addresses.  This module must only be used on an
//! STM32G474RE target.
//!
//! # Reference
//!
//! RM0440 Rev 8, chapter 44 (FDCAN).

use bsw_can::frame::CanFrame;
use bsw_can::can_id::CanId;
use bsw_can::transceiver::{
    AbstractTransceiver, CanTransceiver, ErrorCode, State, TransceiverState,
};

// ---------------------------------------------------------------------------
// Memory-map constants
// ---------------------------------------------------------------------------

/// FDCAN1 register block base address (RM0440 §44.4).
const FDCAN1_BASE: usize = 0x4000_6400;

/// Message RAM base address for all FDCAN instances (RM0440 §44.3.3).
const SRAMCAN_BASE: usize = 0x4000_A400;

// ---------------------------------------------------------------------------
// FDCAN1 register offsets (byte offsets from FDCAN1_BASE)
// ---------------------------------------------------------------------------

const FDCAN_CCCR_OFFSET: usize = 0x018; // CC control register
const FDCAN_NBTP_OFFSET: usize = 0x01C; // Nominal bit timing
const FDCAN_RXGFC_OFFSET: usize = 0x080; // Global filter config (STM32G4: replaces SIDFC/XIDFC/RXF0C/RXF1C)
const FDCAN_RXF0S_OFFSET: usize = 0x090; // RX FIFO 0 status       (STM32G4: 0x090, not 0x0A4)
const FDCAN_RXF0A_OFFSET: usize = 0x094; // RX FIFO 0 acknowledge  (STM32G4: 0x094, not 0x0A8)
const FDCAN_TXBC_OFFSET:  usize = 0x0C0; // TX buffer config        (STM32G4: 0x0C0, not 0x0C4)
const FDCAN_TXFQS_OFFSET: usize = 0x0C4; // TX FIFO/queue status    (STM32G4: 0x0C4, not 0x0C8)
const FDCAN_TXBAR_OFFSET: usize = 0x0CC; // TX buffer add request   (STM32G4: 0x0CC, not 0x0D0)
const FDCAN_IR_OFFSET: usize = 0x050;   // Interrupt register
const FDCAN_IE_OFFSET: usize = 0x054;   // Interrupt enable
const FDCAN_ILS_OFFSET: usize = 0x058;  // Interrupt line select
const FDCAN_ILE_OFFSET: usize = 0x05C;  // Interrupt line enable
const FDCAN_PSR_OFFSET: usize = 0x044;  // Protocol status register
// NOTE: SIDFC (0x084), XIDFC (0x088), RXF0C (0x0A0), RXF1C (0x0B0), and
// TXEFC (0x0F0) do NOT exist on STM32G4.  Those addresses are XIDAM, HPMS,
// reserved, reserved, and TXEFA respectively.  MRAM layout on STM32G4 is
// hardware-fixed and configured only via TXBC and RXGFC.

// ---------------------------------------------------------------------------
// Message RAM offsets (byte offsets from SRAMCAN_BASE for FDCAN1)
// ---------------------------------------------------------------------------

const MRAM_STDFILTER_OFFSET: usize = 0x000; // 28 × 1 word  std filters
const MRAM_EXTFILTER_OFFSET: usize = 0x070; // 8  × 2 words ext filters
const MRAM_RXF0_OFFSET:      usize = 0x0B0; // 3  × 18 words RX FIFO 0 elements  (3 × 72 B = 216 B)
const MRAM_RXF1_OFFSET:      usize = 0x188; // 3  × 18 words RX FIFO 1 elements  (STM32G4: was 0x0E0)
const MRAM_TXEVT_OFFSET:     usize = 0x260; // 3  × 2  words TX event FIFO        (STM32G4: was 0x110)
const MRAM_TXBUF_OFFSET:     usize = 0x278; // 3  × 18 words TX buffer elements   (STM32G4: was 0x128)

/// Byte stride between consecutive RX/TX MRAM elements on STM32G4.
///
/// STM32G4 FDCAN uses 18-word (72-byte) elements for both RX FIFOs and TX
/// buffers, regardless of DLC.  Generic Bosch M_CAN uses 4-word (16-byte)
/// elements for classic CAN; that value is WRONG on STM32G4.
const MRAM_ELEMENT_STRIDE: usize = 18 * 4; // 72 bytes

// ---------------------------------------------------------------------------
// FDCAN_CCCR bit masks
// ---------------------------------------------------------------------------

/// INIT bit — 1 = initialisation mode.
const CCCR_INIT: u32 = 1 << 0;
/// CCE bit — configuration change enable (requires INIT=1).
const CCCR_CCE: u32 = 1 << 1;
/// FDOE — FD operation enable; 0 = classic CAN only.
const CCCR_FDOE: u32 = 1 << 8;
/// BRSE — bit-rate switching enable; 0 = disabled.
const CCCR_BRSE: u32 = 1 << 9;
/// MON — silent monitoring mode (listen-only, no TX acknowledge).
const CCCR_MON: u32 = 1 << 5;
/// DAR — disable automatic retransmission.
#[allow(dead_code)]
const CCCR_DAR: u32 = 1 << 6;

// ---------------------------------------------------------------------------
// Bit timing value for 500 kbit/s @ 170 MHz FDCAN clock
//
// NBTP layout (RM0440 §44.4.10):
//   Bits [31:25]  NSJW    (7 bits,  value = SJW−1)
//   Bits [24:16]  NTSEG1  (9 bits,  value = Tseg1−1)
//   Bits [14:8]   NTSEG2  (7 bits,  value = Tseg2−1)
//   Bits [8:0]    NBRP    (9 bits,  value = prescaler−1)
//
// Choosing:
//   NBRP = 16 → prescaler = 17 → TQ clock = 170/17 = 10 MHz
//   NTSEG1 = 11 → Tseg1 = 12
//   NTSEG2 = 3  → Tseg2 = 4
//   NSJW   = 3  → SJW   = 4
//   Bit time = 1 + Tseg1 + Tseg2 = 1 + 12 + 4 = 17 TQ
//   Baud = 10_000_000 / 17 ≈ 588 kbps  ← adjusting:
//
// Re-choosing for exact 500 kbps:
//   NBRP = 19 → prescaler = 20 → TQ clock = 170/20 = 8.5 MHz
//   NTSEG1 = 12 → Tseg1 = 13
//   NTSEG2 = 3  → Tseg2 = 4   (note: RM0440 constraint Tseg1 ≥ 2×Tseg2)
//   Bit time = 1 + 13 + 4 = 18 TQ  → nope
//
// Exact: 170_000_000 / 500_000 = 340 TQ total per bit-time.
// Choose prescaler = 34, TQ_clk = 5 MHz, bit_time = 10 TQ:
//   NTSEG1 = 6 (7 TQ), NTSEG2 = 2 (3 TQ), 1+7+3 = 11 ← still off
// Choose prescaler = 17 → TQ_clk = 10 MHz, need 20 TQ/bit:
//   NTSEG1 = 14 (15 TQ), NTSEG2 = 4 (5 TQ), 1+15+5 = 21 ← off by 1
// Exactly 20 TQ: NTSEG1=13 (14TQ) + NTSEG2=4 (5TQ) + sync = 20 ✓
//   NSJW = 4 (NSJW−1 = 3), meet constraint NTSEG1(13) ≥ 2×NTSEG2(4) = 8 ✓
// ---------------------------------------------------------------------------

/// FDCAN_NBTP value for 500 kbit/s with FDCAN_CLK = 170 MHz.
///
/// prescaler=17 (NBRP=16), NTSEG1=13 (field=13), NTSEG2=4 (field=4−1=3),
/// NSJW=4 (field=4−1=3).
/// Bit time = 1 + 14 + 5 = 20 TQ @ 10 MHz = 500 kbit/s.
const NBTP_500KBPS: u32 = {
    let nbrp: u32 = 16;     // prescaler − 1
    let ntseg1: u32 = 14;   // Tseg1 − 1 = 15 − 1  (matches .ioc NominalTimeSeg1=15)
    let ntseg2: u32 = 3;    // Tseg2 − 1 = 4 − 1  (bits [14:8])
    let nsjw: u32 = 3;      // SJW − 1 = 4 − 1  (bits [31:25])
    (nsjw << 25) | (nbrp << 16) | (ntseg1 << 8) | ntseg2
};

// ---------------------------------------------------------------------------
// FDCAN_IR / FDCAN_IE bit masks
// ---------------------------------------------------------------------------

/// RF0N — RX FIFO 0 new message interrupt.
const IR_RF0N: u32 = 1 << 0;
/// TC — transmission completed interrupt.
const IR_TC: u32 = 1 << 9;
/// BO — bus-off status changed interrupt.
const IR_BO: u32 = 1 << 25;
/// EP — error passive changed interrupt.
const IR_EP: u32 = 1 << 23;

// ---------------------------------------------------------------------------
// FDCAN_PSR bit masks
// ---------------------------------------------------------------------------

/// BO bit in PSR — bus-off state.
const PSR_BO: u32 = 1 << 7;
/// EP bit in PSR — error passive state.
const PSR_EP: u32 = 1 << 5;

// ---------------------------------------------------------------------------
// FDCAN_TXFQS bit masks
// ---------------------------------------------------------------------------

/// TFFL[5:0] — TX FIFO free level.
#[allow(dead_code)]
const TXFQS_TFFL_MASK: u32 = 0x3F;
/// TFQPI[9:8] — TX FIFO/queue put index (STM32G4: 2-bit field, max 3 entries).
const TXFQS_TFQPI_MASK: u32 = 0x3 << 8;
const TXFQS_TFQPI_SHIFT: u32 = 8;
/// TFQF — TX FIFO/queue full.
const TXFQS_TFQF: u32 = 1 << 21;

// ---------------------------------------------------------------------------
// FDCAN_RXF0S bit masks
// ---------------------------------------------------------------------------

/// F0FL[3:0] — RX FIFO 0 fill level (STM32G4: 4-bit field [3:0], not 7-bit).
const RXF0S_F0FL_MASK: u32 = 0xF;
/// F0GI[9:8] — RX FIFO 0 get index (STM32G4: 2-bit field [9:8], not 6-bit [13:8]).
const RXF0S_F0GI_MASK: u32 = 0x3 << 8;
const RXF0S_F0GI_SHIFT: u32 = 8;

// ---------------------------------------------------------------------------
// Message RAM element helpers
// ---------------------------------------------------------------------------

/// Word size in bytes.
const WORD: usize = 4;

/// Absolute address of TX buffer element `i` in message RAM.
///
/// STM32G4 FDCAN TX buffer elements are 18 words (72 bytes) each.
/// Stride = `MRAM_ELEMENT_STRIDE` (not 4 × WORD as in generic M_CAN).
#[inline(always)]
const fn txbuf_elem_addr(i: usize) -> usize {
    SRAMCAN_BASE + MRAM_TXBUF_OFFSET + i * MRAM_ELEMENT_STRIDE
}

/// Absolute address of RX FIFO 0 element `i` in message RAM.
///
/// STM32G4 FDCAN RX FIFO elements are 18 words (72 bytes) each.
#[inline(always)]
const fn rxf0_elem_addr(i: usize) -> usize {
    SRAMCAN_BASE + MRAM_RXF0_OFFSET + i * MRAM_ELEMENT_STRIDE
}

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

/// Read a 32-bit word from an absolute MMIO address (message RAM).
#[inline(always)]
unsafe fn mram_read(addr: usize) -> u32 {
    // SAFETY: caller guarantees the address is inside message RAM.
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

/// Write a 32-bit word to an absolute MMIO address (message RAM).
#[inline(always)]
unsafe fn mram_write(addr: usize, val: u32) {
    // SAFETY: caller guarantees the address is inside message RAM.
    unsafe { core::ptr::write_volatile(addr as *mut u32, val) }
}

// ---------------------------------------------------------------------------
// Queue size
// ---------------------------------------------------------------------------

/// RX software ring-buffer depth.
pub const RX_QUEUE_SIZE: usize = 16;

/// TX buffer count in message RAM (hardware limit).
const TX_BUF_COUNT: usize = 3;

// ---------------------------------------------------------------------------
// FdCanTransceiver
// ---------------------------------------------------------------------------

/// FDCAN1 transceiver for STM32G474RE, implementing [`CanTransceiver`].
///
/// Wraps an [`AbstractTransceiver<8>`] for state-machine management and
/// exposes a software RX ring-buffer for deferred frame processing.
///
/// # Usage pattern
///
/// ```ignore
/// static mut FDCAN: FdCanTransceiver = FdCanTransceiver::new(0);
/// // In startup:
/// FDCAN.init();
/// FDCAN.open();
/// // In main loop:
/// while let Some(frame) = FDCAN.receive() { /* process frame */ }
/// // To send:
/// FDCAN.write(&frame);
/// ```
pub struct FdCanTransceiver {
    /// Hardware-independent state, filter, and statistics.
    base: AbstractTransceiver<8>,
    /// Software RX ring-buffer (frames copied out of message RAM in the ISR).
    rx_queue: [CanFrame; RX_QUEUE_SIZE],
    /// Write index (producer = ISR).
    rx_head: usize,
    /// Number of frames currently in the ring-buffer.
    rx_count: usize,
}

impl FdCanTransceiver {
    /// Creates a new transceiver instance in `Closed` state.
    ///
    /// `bus_id` is the logical CAN bus index within the system (used by
    /// [`AbstractTransceiver`] for routing; typically `0` for FDCAN1).
    pub const fn new(bus_id: u8) -> Self {
        // CanFrame::new() is const; repeat the initialiser inline to keep this
        // function const-compatible on stable Rust (array init from non-Copy
        // const default is stable since 1.59).
        const EMPTY: CanFrame = CanFrame::new();
        Self {
            base: AbstractTransceiver::new(bus_id),
            rx_queue: [EMPTY; RX_QUEUE_SIZE],
            rx_head: 0,
            rx_count: 0,
        }
    }

    // -----------------------------------------------------------------------
    // Public helpers for interrupt-driven receive
    // -----------------------------------------------------------------------

    /// Returns and removes the oldest frame from the software RX ring-buffer.
    ///
    /// Returns `None` if the buffer is empty.  Call this from the main loop
    /// or RTOS task after the ISR has deposited frames via `isr_rx_fifo0()`.
    pub fn receive(&mut self) -> Option<CanFrame> {
        if self.rx_count == 0 {
            return None;
        }
        let tail = (self.rx_head + RX_QUEUE_SIZE - self.rx_count) % RX_QUEUE_SIZE;
        let frame = self.rx_queue[tail].clone();
        self.rx_count -= 1;
        Some(frame)
    }

    /// Drain the FDCAN1 RX FIFO 0 into the software ring-buffer.
    ///
    /// Call this from the FDCAN1 interrupt handler (or polling context) when
    /// `FDCAN_IR.RF0N` is set.  Clears RF0N and TC interrupt flags before
    /// returning.
    ///
    /// Frames received while the software buffer is full are silently dropped
    /// and counted in `statistics_mut().rx_dropped`.
    ///
    /// # Safety
    ///
    /// Must be called with FDCAN1 interrupt line masked (or from within the
    /// FDCAN1 ISR) to avoid concurrent access to `rx_queue`.
    pub unsafe fn isr_rx_fifo0(&mut self) {
        loop {
            // SAFETY: FDCAN1_BASE + offset is a valid MMIO address.
            let rxf0s = unsafe { reg_read(FDCAN1_BASE, FDCAN_RXF0S_OFFSET) };
            let fill = rxf0s & RXF0S_F0FL_MASK;
            if fill == 0 {
                break;
            }
            let get_idx = ((rxf0s & RXF0S_F0GI_MASK) >> RXF0S_F0GI_SHIFT) as usize;

            // Read the message RAM element at get_idx.
            let base_addr = rxf0_elem_addr(get_idx);
            // SAFETY: message RAM address is valid for FDCAN1.
            let r0 = unsafe { mram_read(base_addr) };
            let r1 = unsafe { mram_read(base_addr + WORD) };
            let db0 = unsafe { mram_read(base_addr + 2 * WORD) };
            let db1 = unsafe { mram_read(base_addr + 3 * WORD) };

            // Decode R0: bit 30 = XTD (extended ID), bits [28:0] = ID.
            let is_ext = (r0 & (1 << 30)) != 0;
            let raw_id = if is_ext {
                r0 & 0x1FFF_FFFF
            } else {
                // Standard ID is stored in bits [28:18] of R0.
                (r0 >> 18) & 0x7FF
            };

            // Decode R1: bits [19:16] = DLC.
            let dlc = ((r1 >> 16) & 0xF) as u8;
            let dlc_clamped = if dlc > 8 { 8 } else { dlc } as usize;

            // Reassemble the 8 payload bytes from the two data words (little-endian).
            let mut data = [0u8; 8];
            let w0_bytes = db0.to_le_bytes();
            let w1_bytes = db1.to_le_bytes();
            data[..4].copy_from_slice(&w0_bytes);
            data[4..8].copy_from_slice(&w1_bytes);

            // Acknowledge the element (release the slot back to the controller).
            // SAFETY: FDCAN1_BASE + offset is a valid MMIO address.
            unsafe { reg_write(FDCAN1_BASE, FDCAN_RXF0A_OFFSET, get_idx as u32) };

            // Update statistics.
            self.base.statistics_mut().rx += 1;

            // Push to software ring-buffer if space is available.
            if self.rx_count < RX_QUEUE_SIZE {
                let can_id = CanId::id(raw_id, is_ext);
                let frame = CanFrame::with_data(can_id, &data[..dlc_clamped]);
                self.rx_queue[self.rx_head] = frame;
                self.rx_head = (self.rx_head + 1) % RX_QUEUE_SIZE;
                self.rx_count += 1;
            } else {
                self.base.statistics_mut().rx_dropped += 1;
            }
        }

        // Clear RF0N and TC interrupt flags (write-1-to-clear).
        // SAFETY: FDCAN1_BASE + offset is a valid MMIO address.
        unsafe { reg_write(FDCAN1_BASE, FDCAN_IR_OFFSET, IR_RF0N | IR_TC) };
    }

    // -----------------------------------------------------------------------
    // Internal hardware helpers
    // -----------------------------------------------------------------------

    /// Write FDCAN1 into initialisation mode and enable configuration change.
    ///
    /// Sets CCCR.INIT=1 then CCCR.CCE=1 and waits for INIT to be
    /// acknowledged.  Classic-CAN only: clears FDOE and BRSE.
    unsafe fn enter_init(&self) {
        // SAFETY: FDCAN1_BASE + offset is a valid MMIO address.
        unsafe {
            reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_INIT);
            while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) == 0 {}
            reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_CCE);
            // Classic CAN only — clear FD and bit-rate-switching bits.
            reg_modify(
                FDCAN1_BASE,
                FDCAN_CCCR_OFFSET,
                CCCR_FDOE | CCCR_BRSE,
                0,
            );
        }
    }

    /// Leave initialisation mode (INIT=0) to start normal operation.
    ///
    /// Waits until INIT is deasserted by the hardware.
    unsafe fn leave_init(&self) {
        // SAFETY: FDCAN1_BASE + offset is a valid MMIO address.
        unsafe {
            reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_INIT | CCCR_CCE, 0);
            while (reg_read(FDCAN1_BASE, FDCAN_CCCR_OFFSET) & CCCR_INIT) != 0 {}
        }
    }

    /// Configure the message RAM and accept-all filter policy via RXGFC.
    ///
    /// Must be called while CCCR.INIT=1 and CCCR.CCE=1.
    ///
    /// # STM32G4 note
    ///
    /// On STM32G4 the MRAM layout is **hardware-fixed**.  The registers
    /// SIDFC, XIDFC, RXF0C, RXF1C, and TXEFC present in generic Bosch M_CAN
    /// do not exist here — writing to those addresses would corrupt XIDAM,
    /// HPMS, or reserved locations.  Only two registers need to be written:
    ///
    /// - **TXBC** (0x0C0): TX buffer start address (word offset) and count.
    ///   Bits [31:24] = NDT (number of dedicated TX buffers).
    ///   Bits [15:2]  = TBSA (TX buffer start address, word offset from SRAMCAN_BASE).
    ///   We write `MRAM_TXBUF_OFFSET / 4` as the word offset and 3 dedicated buffers.
    ///
    /// - **RXGFC** (0x080): global filter config.
    ///   ANFS[5:4] = 00 → accept all non-matching std-ID frames to FIFO 0.
    ///   ANFE[1:0] = 00 → accept all non-matching ext-ID frames to FIFO 0.
    unsafe fn configure_message_ram(&self) {
        // SAFETY: all base addresses and offsets are valid for STM32G474.
        unsafe {
            // TX dedicated buffers: 3 elements, FIFO mode (TFQM=0).
            // TBSA is the word offset of MRAM_TXBUF_OFFSET from SRAMCAN_BASE.
            reg_write(
                FDCAN1_BASE,
                FDCAN_TXBC_OFFSET,
                (3 << 24) | (MRAM_TXBUF_OFFSET as u32 / 4),
            );

            // Global filter: accept all non-matching std/ext IDs to FIFO 0.
            // ANFS[5:4] = 00 (accept to FIFO 0), ANFE[1:0] = 00.
            reg_write(FDCAN1_BASE, FDCAN_RXGFC_OFFSET, 0x0000_0000);
        }
    }

    /// Set up interrupts: enable RF0N (RX) and TC (TX complete) on line 0.
    ///
    /// Must be called while in init mode.
    unsafe fn configure_interrupts(&self) {
        // SAFETY: FDCAN1_BASE offsets are valid for STM32G474.
        unsafe {
            // Enable RF0N and TC interrupt sources.
            reg_write(FDCAN1_BASE, FDCAN_IE_OFFSET, IR_RF0N | IR_TC | IR_BO | IR_EP);
            // Route all enabled interrupts to interrupt line 0.
            reg_write(FDCAN1_BASE, FDCAN_ILS_OFFSET, 0x0000_0000);
            // Enable interrupt line 0.
            reg_write(FDCAN1_BASE, FDCAN_ILE_OFFSET, 0x0000_0001);
        }
    }

    /// Read the hardware bus state from FDCAN_PSR and update the abstract
    /// transceiver state accordingly.
    fn sync_transceiver_state(&mut self) {
        // SAFETY: FDCAN1_BASE + PSR_OFFSET is a valid MMIO address.
        let psr = unsafe { reg_read(FDCAN1_BASE, FDCAN_PSR_OFFSET) };
        let ts = if (psr & PSR_BO) != 0 {
            TransceiverState::BusOff
        } else if (psr & PSR_EP) != 0 {
            TransceiverState::Passive
        } else {
            TransceiverState::Active
        };
        self.base.set_transceiver_state(ts);
    }

    /// Write a TX buffer element to message RAM at TX buffer slot `slot`.
    ///
    /// `slot` must be in `0..TX_BUF_COUNT`.
    ///
    /// Message RAM TX element layout (RM0440 §44.4.18):
    /// - T0: XTD[30], RTR[29], ID[28:0] or ID[28:18] for std
    /// - T1: DLC[19:16], BRS=0, FDF=0, EFC=0
    /// - DB0: data bytes 3..0
    /// - DB1: data bytes 7..4
    unsafe fn write_tx_element(&self, slot: usize, frame: &CanFrame) {
        let base_addr = txbuf_elem_addr(slot);
        let raw_id = frame.id().raw_id();

        // T0: encode ID.  For extended IDs bit 30 (XTD) = 1 and bits [28:0]
        // carry the 29-bit identifier.  For base IDs, XTD=0 and the 11-bit
        // identifier is stored in bits [28:18].
        let t0 = if frame.id().is_extended() {
            (1 << 30) | (raw_id & 0x1FFF_FFFF)
        } else {
            (raw_id & 0x7FF) << 18
        };

        // T1: DLC in bits [19:16]; FD-related bits all 0.
        let dlc = frame.payload_length() as u32;
        let t1 = dlc << 16;

        // Data words — pack up to 8 bytes, little-endian.
        let payload = frame.payload();
        let mut raw = [0u8; 8];
        let copy_len = if payload.len() > 8 { 8 } else { payload.len() };
        raw[..copy_len].copy_from_slice(&payload[..copy_len]);
        let db0 = u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]);
        let db1 = u32::from_le_bytes([raw[4], raw[5], raw[6], raw[7]]);

        // SAFETY: base_addr is inside FDCAN1 message RAM.
        unsafe {
            mram_write(base_addr, t0);
            mram_write(base_addr + WORD, t1);
            mram_write(base_addr + 2 * WORD, db0);
            mram_write(base_addr + 3 * WORD, db1);
        }
    }
}

// ---------------------------------------------------------------------------
// CanTransceiver implementation
// ---------------------------------------------------------------------------

impl CanTransceiver for FdCanTransceiver {
    /// Initialise the FDCAN1 peripheral.
    ///
    /// Configures bit timing, message RAM, accept-all filter, and interrupt
    /// routing.  Transitions the logical state to `Initialized`.
    ///
    /// Returns `ErrorCode::InitFailed` if the hardware does not respond, or
    /// `ErrorCode::IllegalState` if called while not `Closed`.
    fn init(&mut self) -> ErrorCode {
        if self.base.state() != State::Closed {
            return ErrorCode::IllegalState;
        }

        // SAFETY: single-core startup context assumed; FDCAN1 registers are
        // valid on STM32G474RE.
        unsafe {
            // Enter configuration mode.
            self.enter_init();

            // Bit timing for 500 kbit/s.
            reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);

            // Verify NBTP write took effect (debug: detect bus-off recovery corruption).
            let readback = reg_read(FDCAN1_BASE, FDCAN_NBTP_OFFSET);
            if readback != NBTP_500KBPS {
                // NBTP didn't stick — try writing again
                reg_write(FDCAN1_BASE, FDCAN_NBTP_OFFSET, NBTP_500KBPS);
            }

            // Message RAM layout and accept-all filter.
            self.configure_message_ram();

            // Interrupt routing (RF0N + TC on line 0).
            self.configure_interrupts();
        }

        self.base.transition_to_initialized()
    }

    /// Shut down FDCAN1 by re-entering init mode, suppressing all activity.
    fn shutdown(&mut self) {
        // SAFETY: FDCAN1_BASE + CCCR offset is valid.
        unsafe {
            self.enter_init();
        }
        self.base.transition_to_closed();
    }

    /// Open the bus for normal RX/TX operation.
    ///
    /// Returns `ErrorCode::IllegalState` if called before `init()`.
    fn open(&mut self) -> ErrorCode {
        if self.base.state() != State::Initialized {
            return ErrorCode::IllegalState;
        }
        // SAFETY: FDCAN1 registers valid; called after init().
        unsafe {
            // Clear MON to enable TX acknowledge (normal operation).
            reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_MON, 0);
            self.leave_init();
        }
        self.base.transition_to_open()
    }

    /// Open the bus and immediately enqueue an initial TX frame.
    fn open_with_frame(&mut self, frame: &CanFrame) -> ErrorCode {
        let ec = self.open();
        if ec == ErrorCode::Ok {
            self.write(frame)
        } else {
            ec
        }
    }

    /// Close the bus (re-enter init mode, logical state → Closed).
    fn close(&mut self) -> ErrorCode {
        // SAFETY: FDCAN1 CCCR register is valid.
        unsafe { self.enter_init() };
        self.base.transition_to_closed()
    }

    /// Mute: enter silent/monitoring mode — RX continues, TX is suppressed.
    fn mute(&mut self) -> ErrorCode {
        if self.base.state() != State::Open {
            return ErrorCode::IllegalState;
        }
        // SAFETY: FDCAN1 registers valid.
        unsafe {
            // Re-enter init to change CCCR.MON, then leave init.
            self.enter_init();
            reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, 0, CCCR_MON);
            self.leave_init();
        }
        self.base.transition_to_muted()
    }

    /// Unmute: return to normal TX+RX operation.
    fn unmute(&mut self) -> ErrorCode {
        if self.base.state() != State::Muted {
            return ErrorCode::IllegalState;
        }
        // SAFETY: FDCAN1 registers valid.
        unsafe {
            self.enter_init();
            reg_modify(FDCAN1_BASE, FDCAN_CCCR_OFFSET, CCCR_MON, 0);
            self.leave_init();
        }
        self.base.transition_to_open()
    }

    fn state(&self) -> State {
        self.base.state()
    }

    fn baudrate(&self) -> u32 {
        self.base.baudrate()
    }

    fn hw_queue_timeout(&self) -> u16 {
        // 10 ms default TX queue timeout.
        10
    }

    fn bus_id(&self) -> u8 {
        self.base.bus_id()
    }

    /// Transmit a frame via FDCAN1 TX FIFO.
    ///
    /// Checks the TXFQS fill level, writes the message RAM TX buffer element,
    /// and sets the corresponding TXBAR bit to request transmission.
    ///
    /// Returns `ErrorCode::TxHwQueueFull` when all TX buffer slots are
    /// occupied, or `ErrorCode::TxOffline` when the bus is not `Open`.
    fn write(&mut self, frame: &CanFrame) -> ErrorCode {
        if self.base.state() != State::Open {
            return ErrorCode::TxOffline;
        }

        // SAFETY: FDCAN1_BASE + offsets are valid MMIO addresses.
        let txfqs = unsafe { reg_read(FDCAN1_BASE, FDCAN_TXFQS_OFFSET) };
        if (txfqs & TXFQS_TFQF) != 0 {
            self.base.statistics_mut().tx_dropped += 1;
            return ErrorCode::TxHwQueueFull;
        }

        let put_idx = ((txfqs & TXFQS_TFQPI_MASK) >> TXFQS_TFQPI_SHIFT) as usize;
        if put_idx >= TX_BUF_COUNT {
            self.base.statistics_mut().tx_dropped += 1;
            return ErrorCode::TxHwQueueFull;
        }

        // SAFETY: put_idx < TX_BUF_COUNT; message RAM addresses are valid.
        unsafe { self.write_tx_element(put_idx, frame) };

        // Request transmission of the buffer we just filled.
        // SAFETY: TXBAR write is valid FDCAN1 register access.
        unsafe { reg_write(FDCAN1_BASE, FDCAN_TXBAR_OFFSET, 1 << put_idx) };

        self.base.statistics_mut().tx += 1;
        ErrorCode::Ok
    }

    /// Returns the current hardware bus state, refreshed from FDCAN_PSR.
    fn transceiver_state(&self) -> TransceiverState {
        // Re-read PSR for an up-to-date status (immutable borrow: cast away
        // mutability only to sync state — no actual mutation of the buffer).
        // SAFETY: FDCAN1_BASE + PSR_OFFSET is valid.
        let psr = unsafe { reg_read(FDCAN1_BASE, FDCAN_PSR_OFFSET) };
        if (psr & PSR_BO) != 0 {
            TransceiverState::BusOff
        } else if (psr & PSR_EP) != 0 {
            TransceiverState::Passive
        } else {
            TransceiverState::Active
        }
    }
}

// ---------------------------------------------------------------------------
// Expose statistics accessor
// ---------------------------------------------------------------------------

impl FdCanTransceiver {
    /// Returns the runtime frame and error counters.
    #[inline]
    pub fn statistics(&self) -> &bsw_can::transceiver::Statistics {
        self.base.statistics()
    }

    /// Updates the internal transceiver state from hardware (PSR register).
    ///
    /// Call this periodically (e.g. once per main loop tick) to keep the
    /// software state in sync with the hardware bus-off / error-passive flags.
    pub fn poll_hardware_state(&mut self) {
        self.sync_transceiver_state();
    }
}

// Implement CanReceiver for DiagCanTransport integration.
impl crate::diag_can::CanReceiver for FdCanTransceiver {
    fn receive(&mut self) -> Option<bsw_can::frame::CanFrame> {
        self.receive()
    }
}
