// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! COM manager — owns the signal shadow buffers and PDU state.
//!
//! [`ComManager`] is the central entity of the COM layer.  Application code
//! calls [`write_signal`](ComManager::write_signal) /
//! [`read_signal`](ComManager::read_signal) to exchange typed signal values.
//! The manager handles packing/unpacking to/from PDU shadow buffers and drives
//! cyclic TX scheduling via [`tick`](ComManager::tick).
//!
//! # No-std / no-heap
//!
//! All storage is inline and statically sized via const generics
//! (`MAX_PDUS`, `MAX_SIGNALS`).  No allocator is required.

use crate::packer::{pack_signal, unpack_signal};
use crate::pdu::{PduDescriptor, PduDirection};
use crate::signal::{SignalDescriptor, SignalValue};

// ---------------------------------------------------------------------------
// ComTxIterator
// ---------------------------------------------------------------------------

/// Iterator yielding `(can_id, data)` pairs for PDUs ready to be transmitted.
///
/// Produced by [`ComManager::tick`].  The lifetime `'m` is tied to the
/// [`ComManager`] borrow so the data slices are valid for the duration of
/// the iteration.
pub struct ComTxIterator<'m> {
    pdus: &'m [PduDescriptor],
    buffers: &'m [[u8; 8]],
    pdu_count: usize,
    ready: &'m [bool],
    index: usize,
}

impl<'m> Iterator for ComTxIterator<'m> {
    type Item = (u32, &'m [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        while self.index < self.pdu_count {
            let i = self.index;
            self.index += 1;
            if self.ready[i] && self.pdus[i].direction == PduDirection::Tx {
                let pdu = &self.pdus[i];
                let data = &self.buffers[i][..pdu.length as usize];
                return Some((pdu.can_id, data));
            }
        }
        None
    }
}

// ---------------------------------------------------------------------------
// ComManager
// ---------------------------------------------------------------------------

/// COM manager.
///
/// # Type parameters
///
/// - `MAX_PDUS` — maximum number of PDUs that can be registered.
/// - `MAX_SIGNALS` — maximum total signals across all PDUs.
pub struct ComManager<const MAX_PDUS: usize, const MAX_SIGNALS: usize> {
    /// Registered PDU descriptors.
    pdus: [PduDescriptor; MAX_PDUS],
    /// Registered signal descriptors.
    signals: [SignalDescriptor; MAX_SIGNALS],
    /// Number of registered PDUs.
    pdu_count: usize,
    /// Number of registered signals.
    signal_count: usize,
    /// Shadow buffer for each PDU (packed bytes, 8 bytes max classic CAN).
    pdu_buffers: [[u8; 8]; MAX_PDUS],
    /// Timestamp (ms) when the next TX is due for each PDU.
    tx_deadlines: [u32; MAX_PDUS],
    /// Timestamp (ms) of the last RX for each PDU (0 = never received).
    rx_timestamps: [u32; MAX_PDUS],
    /// Scratch array: which PDUs are ready to TX on the current tick.
    tx_ready: [bool; MAX_PDUS],
}

// Const-default sentinel PDU/signal for array initialisation.
const SENTINEL_PDU: PduDescriptor = PduDescriptor {
    can_id: 0,
    length: 0,
    direction: PduDirection::Tx,
    cycle_time_ms: 0,
    signal_count: 0,
    signal_start_index: 0,
};

use crate::signal::{ByteOrder, SignalType};
const SENTINEL_SIGNAL: SignalDescriptor = SignalDescriptor {
    id: u16::MAX,
    bit_position: 0,
    bit_size: 8,
    signal_type: SignalType::Uint8,
    byte_order: ByteOrder::LittleEndian,
    init_value: 0,
};

impl<const MAX_PDUS: usize, const MAX_SIGNALS: usize> ComManager<MAX_PDUS, MAX_SIGNALS> {
    // ---------------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------------

    /// Create an empty COM manager (all-zero state).
    pub const fn new() -> Self {
        Self {
            pdus: [SENTINEL_PDU; MAX_PDUS],
            signals: [SENTINEL_SIGNAL; MAX_SIGNALS],
            pdu_count: 0,
            signal_count: 0,
            pdu_buffers: [[0u8; 8]; MAX_PDUS],
            tx_deadlines: [0u32; MAX_PDUS],
            rx_timestamps: [0u32; MAX_PDUS],
            tx_ready: [false; MAX_PDUS],
        }
    }

    // ---------------------------------------------------------------------------
    // Registration
    // ---------------------------------------------------------------------------

    /// Register a PDU together with its signals.
    ///
    /// Returns the assigned PDU index (0-based) wrapped in `Some`, or `None`
    /// if there is no room for the PDU or its signals.
    ///
    /// The signals slice is appended to the internal signal table in order;
    /// `pdu.signal_start_index` is **overwritten** with the actual start index.
    pub fn add_pdu(
        &mut self,
        mut pdu: PduDescriptor,
        signals: &[SignalDescriptor],
    ) -> Option<u16> {
        if self.pdu_count >= MAX_PDUS {
            return None;
        }
        if self.signal_count + signals.len() > MAX_SIGNALS {
            return None;
        }

        let pdu_index = self.pdu_count;
        let sig_start = self.signal_count;

        // Append signals, replacing each descriptor's id with its actual id
        // (no re-mapping needed — caller provides real IDs).
        for sig in signals {
            self.signals[self.signal_count] = *sig;
            self.signal_count += 1;
        }

        // Rewrite signal_start_index and signal_count to actual positions.
        pdu.signal_start_index = sig_start as u16;
        pdu.signal_count = signals.len() as u8;

        // Apply signal init values into the shadow buffer.
        for i in sig_start..self.signal_count {
            let desc = &self.signals[i];
            let init = SignalValue::from_u32(desc.init_value, desc.signal_type);
            pack_signal(&mut self.pdu_buffers[pdu_index], desc, init);
        }

        self.pdus[pdu_count_index(pdu_index)] = pdu;
        self.pdu_count += 1;

        Some(pdu_index as u16)
    }

    // ---------------------------------------------------------------------------
    // Signal access
    // ---------------------------------------------------------------------------

    /// Write `value` to the signal's shadow buffer slot inside its PDU.
    ///
    /// Returns `true` if the signal was found and written, `false` otherwise.
    pub fn write_signal(&mut self, signal_id: u16, value: SignalValue) -> bool {
        let Some(sig_idx) = self.find_signal_index(signal_id) else {
            return false;
        };
        let desc = self.signals[sig_idx];

        let Some(pdu_idx) = self.pdu_for_signal(sig_idx) else {
            return false;
        };

        pack_signal(&mut self.pdu_buffers[pdu_idx], &desc, value);
        true
    }

    /// Read the current shadow-buffer value of a signal.
    ///
    /// Returns `None` if no signal with `signal_id` is registered.
    pub fn read_signal(&self, signal_id: u16) -> Option<SignalValue> {
        let sig_idx = self.find_signal_index(signal_id)?;
        let desc = &self.signals[sig_idx];
        let pdu_idx = self.pdu_for_signal(sig_idx)?;
        Some(unpack_signal(&self.pdu_buffers[pdu_idx], desc))
    }

    // ---------------------------------------------------------------------------
    // Cyclic tick
    // ---------------------------------------------------------------------------

    /// Advance the COM scheduler to `now_ms`.
    ///
    /// For every cyclic TX PDU whose deadline has elapsed, the PDU is marked
    /// ready and its deadline is advanced by `cycle_time_ms`.
    ///
    /// Returns a [`ComTxIterator`] over all PDUs that are due for transmission
    /// on this tick.  **Consume the iterator before the next call to `tick`.**
    pub fn tick(&mut self, now_ms: u32) -> ComTxIterator<'_> {
        // Reset ready flags.
        for flag in &mut self.tx_ready[..self.pdu_count] {
            *flag = false;
        }

        for i in 0..self.pdu_count {
            let pdu = &self.pdus[i];
            if pdu.direction != PduDirection::Tx || pdu.cycle_time_ms == 0 {
                continue;
            }
            if now_ms >= self.tx_deadlines[i] {
                self.tx_ready[i] = true;
                // Advance deadline; wrap-safe because u32 wraps.
                self.tx_deadlines[i] =
                    now_ms.wrapping_add(u32::from(pdu.cycle_time_ms));
            }
        }

        ComTxIterator {
            pdus: &self.pdus,
            buffers: &self.pdu_buffers,
            pdu_count: self.pdu_count,
            ready: &self.tx_ready,
            index: 0,
        }
    }

    // ---------------------------------------------------------------------------
    // RX processing
    // ---------------------------------------------------------------------------

    /// Process a received CAN frame: unpack all signals for the matching PDU
    /// into the shadow buffer and record the receive timestamp.
    ///
    /// Does nothing if no registered PDU has `can_id`.
    pub fn receive(&mut self, can_id: u32, data: &[u8], now_ms: u32) {
        for i in 0..self.pdu_count {
            if self.pdus[i].can_id == can_id && self.pdus[i].direction == PduDirection::Rx {
                // Copy incoming bytes into shadow buffer (up to PDU length).
                let pdu_len = self.pdus[i].length as usize;
                let copy_len = data.len().min(pdu_len).min(8);
                self.pdu_buffers[i][..copy_len].copy_from_slice(&data[..copy_len]);
                self.rx_timestamps[i] = now_ms;
                return;
            }
        }
    }

    // ---------------------------------------------------------------------------
    // RX timeout
    // ---------------------------------------------------------------------------

    /// Iterate over CAN IDs of RX PDUs whose last receive timestamp is older
    /// than `timeout_ms` relative to `now_ms`.
    ///
    /// PDUs that have never been received (timestamp == 0 and now_ms > 0) are
    /// also considered timed-out.
    pub fn check_rx_timeouts(
        &self,
        now_ms: u32,
        timeout_ms: u32,
    ) -> impl Iterator<Item = u32> + '_ {
        (0..self.pdu_count).filter_map(move |i| {
            if self.pdus[i].direction != PduDirection::Rx {
                return None;
            }
            let last = self.rx_timestamps[i];
            // Wrapping subtraction handles u32 rollover.
            if now_ms.wrapping_sub(last) > timeout_ms {
                Some(self.pdus[i].can_id)
            } else {
                None
            }
        })
    }

    // ---------------------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------------------

    fn find_signal_index(&self, signal_id: u16) -> Option<usize> {
        (0..self.signal_count).find(|&i| self.signals[i].id == signal_id)
    }

    fn pdu_for_signal(&self, sig_idx: usize) -> Option<usize> {
        for p in 0..self.pdu_count {
            let start = self.pdus[p].signal_start_index as usize;
            let end = start + self.pdus[p].signal_count as usize;
            if (start..end).contains(&sig_idx) {
                return Some(p);
            }
        }
        None
    }
}

/// Trivial identity — avoids a clippy warning about indexing with `usize`
/// expressions directly while keeping the code readable.
#[inline(always)]
const fn pdu_count_index(i: usize) -> usize {
    i
}

// ---------------------------------------------------------------------------
// Default
// ---------------------------------------------------------------------------

impl<const MAX_PDUS: usize, const MAX_SIGNALS: usize> Default
    for ComManager<MAX_PDUS, MAX_SIGNALS>
{
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::ComManager;
    use crate::pdu::{PduDescriptor, PduDirection};
    use crate::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};

    // Helper: build a standard 8-byte TX PDU descriptor (add_pdu rewrites
    // signal_start_index, so the value here doesn't matter).
    fn tx_pdu(can_id: u32, cycle_ms: u16) -> PduDescriptor {
        PduDescriptor {
            can_id,
            length: 8,
            direction: PduDirection::Tx,
            cycle_time_ms: cycle_ms,
            signal_count: 0,
            signal_start_index: 0,
        }
    }

    fn rx_pdu(can_id: u32) -> PduDescriptor {
        PduDescriptor {
            can_id,
            length: 8,
            direction: PduDirection::Rx,
            cycle_time_ms: 0,
            signal_count: 0,
            signal_start_index: 0,
        }
    }

    fn u8_signal(id: u16, byte: u16, init: u32) -> SignalDescriptor {
        SignalDescriptor {
            id,
            bit_position: byte * 8,
            bit_size: 8,
            signal_type: SignalType::Uint8,
            byte_order: ByteOrder::LittleEndian,
            init_value: init,
        }
    }

    fn u16_signal(id: u16, byte: u16) -> SignalDescriptor {
        SignalDescriptor {
            id,
            bit_position: byte * 8,
            bit_size: 16,
            signal_type: SignalType::Uint16,
            byte_order: ByteOrder::LittleEndian,
            init_value: 0,
        }
    }

    fn bool_signal(id: u16, byte: u16, init: u32) -> SignalDescriptor {
        SignalDescriptor {
            id,
            bit_position: byte * 8,
            bit_size: 8,
            signal_type: SignalType::Boolean,
            byte_order: ByteOrder::LittleEndian,
            init_value: init,
        }
    }

    // 1 — write/read round-trip for u8 signal
    #[test]
    fn write_read_u8_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(1, 0, 0)];
        com.add_pdu(tx_pdu(0x100, 10), &sigs);
        assert!(com.write_signal(1, SignalValue::U8(0x42)));
        assert_eq!(com.read_signal(1), Some(SignalValue::U8(0x42)));
    }

    // 2 — write/read round-trip for u16 signal
    #[test]
    fn write_read_u16_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u16_signal(2, 0)];
        com.add_pdu(tx_pdu(0x200, 20), &sigs);
        assert!(com.write_signal(2, SignalValue::U16(0xABCD)));
        assert_eq!(com.read_signal(2), Some(SignalValue::U16(0xABCD)));
    }

    // 3 — signal init values are applied at add_pdu time
    #[test]
    fn signal_init_value_applied() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(3, 2, 0xFF)];
        com.add_pdu(tx_pdu(0x300, 5), &sigs);
        assert_eq!(com.read_signal(3), Some(SignalValue::U8(0xFF)));
    }

    // 4 — unknown signal ID returns None on read
    #[test]
    fn unknown_signal_read_returns_none() {
        let com: ComManager<4, 16> = ComManager::new();
        assert_eq!(com.read_signal(999), None);
    }

    // 5 — unknown signal ID returns false on write
    #[test]
    fn unknown_signal_write_returns_false() {
        let mut com: ComManager<4, 16> = ComManager::new();
        assert!(!com.write_signal(999, SignalValue::U8(0)));
    }

    // 6 — cyclic TX: PDU not ready before deadline
    #[test]
    fn cyclic_tx_not_ready_before_deadline() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(10, 0, 0)];
        com.add_pdu(tx_pdu(0x100, 10), &sigs); // cycle = 10 ms

        // At t=0 the deadline is 0, so the first tick fires immediately.
        // Reset deadlines by advancing past first tick.
        let _ = com.tick(0).count(); // fires at t=0
        // Now deadline is 10. At t=5 it should NOT be ready.
        let count = com.tick(5).count();
        assert_eq!(count, 0, "PDU should not be ready before deadline");
    }

    // 7 — cyclic TX: PDU ready at/after deadline
    #[test]
    fn cyclic_tx_ready_at_deadline() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(10, 0, 0)];
        com.add_pdu(tx_pdu(0x100, 10), &sigs);

        let _ = com.tick(0).count(); // first fire, deadline → 10
        let ready: Vec<_> = com.tick(10).collect();
        assert_eq!(ready.len(), 1);
        assert_eq!(ready[0].0, 0x100);
    }

    // 8 — TX iterator returns correct CAN ID and data
    #[test]
    fn tx_iterator_data_correct() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(20, 0, 0xAB)];
        com.add_pdu(tx_pdu(0x123, 10), &sigs);
        let _ = com.tick(0).count(); // first fire
        // advance past deadline for second fire
        let ready: Vec<(u32, Vec<u8>)> = com
            .tick(10)
            .map(|(id, data)| (id, data.to_vec()))
            .collect();
        assert_eq!(ready.len(), 1);
        assert_eq!(ready[0].0, 0x123);
        assert_eq!(ready[0].1[0], 0xAB);
    }

    // 9 — receive updates shadow buffer
    #[test]
    fn receive_updates_shadow_buffer() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(30, 0, 0)];
        com.add_pdu(rx_pdu(0x400), &sigs);
        com.receive(0x400, &[0x55, 0, 0, 0, 0, 0, 0, 0], 100);
        assert_eq!(com.read_signal(30), Some(SignalValue::U8(0x55)));
    }

    // 10 — receive on unknown CAN ID does nothing (no panic)
    #[test]
    fn receive_unknown_can_id_no_panic() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.receive(0xDEAD, &[1, 2, 3, 4, 5, 6, 7, 8], 0); // must not panic
    }

    // 11 — RX timeout detection
    #[test]
    fn rx_timeout_detection() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(40, 0, 0)];
        com.add_pdu(rx_pdu(0x500), &sigs);
        // Receive at t=0, check at t=150 with timeout=100 → timed out.
        com.receive(0x500, &[1, 0, 0, 0, 0, 0, 0, 0], 0);
        let timed_out: Vec<u32> = com.check_rx_timeouts(150, 100).collect();
        assert_eq!(timed_out, vec![0x500]);
    }

    // 12 — RX timeout NOT triggered when within window
    #[test]
    fn rx_no_timeout_within_window() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(41, 0, 0)];
        com.add_pdu(rx_pdu(0x501), &sigs);
        com.receive(0x501, &[1, 0, 0, 0, 0, 0, 0, 0], 100);
        let timed_out: Vec<u32> = com.check_rx_timeouts(150, 100).collect();
        assert!(timed_out.is_empty());
    }

    // 13 — PDU capacity overflow returns None
    #[test]
    fn pdu_capacity_overflow() {
        let mut com: ComManager<2, 32> = ComManager::new();
        assert!(com.add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0)]).is_some());
        assert!(com.add_pdu(tx_pdu(0x200, 20), &[u8_signal(2, 0, 0)]).is_some());
        // 3rd PDU exceeds MAX_PDUS=2.
        assert!(com.add_pdu(tx_pdu(0x300, 30), &[u8_signal(3, 0, 0)]).is_none());
    }

    // 14 — multiple signals in one PDU, all round-trip correctly
    #[test]
    fn multiple_signals_same_pdu_round_trip() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [u8_signal(50, 0, 0), u8_signal(51, 1, 0), u16_signal(52, 2)];
        com.add_pdu(tx_pdu(0x600, 10), &sigs);

        com.write_signal(50, SignalValue::U8(0x11));
        com.write_signal(51, SignalValue::U8(0x22));
        com.write_signal(52, SignalValue::U16(0x3344));

        assert_eq!(com.read_signal(50), Some(SignalValue::U8(0x11)));
        assert_eq!(com.read_signal(51), Some(SignalValue::U8(0x22)));
        assert_eq!(com.read_signal(52), Some(SignalValue::U16(0x3344)));
    }

    // 15 — boolean signal init value and round-trip
    #[test]
    fn boolean_signal_init_and_write() {
        let mut com: ComManager<4, 16> = ComManager::new();
        let sigs = [bool_signal(60, 0, 1)]; // init = true
        com.add_pdu(tx_pdu(0x700, 10), &sigs);
        assert_eq!(com.read_signal(60), Some(SignalValue::Bool(true)));

        com.write_signal(60, SignalValue::Bool(false));
        assert_eq!(com.read_signal(60), Some(SignalValue::Bool(false)));
    }

    // 16 — signal capacity overflow returns None
    #[test]
    fn signal_capacity_overflow() {
        // MAX_SIGNALS = 2; try to add 3 signals in one PDU.
        let mut com: ComManager<4, 2> = ComManager::new();
        let sigs = [u8_signal(1, 0, 0), u8_signal(2, 1, 0), u8_signal(3, 2, 0)];
        assert!(com.add_pdu(tx_pdu(0x100, 10), &sigs).is_none());
    }

    // 17 — two PDUs on different CAN IDs are independent
    #[test]
    fn two_pdus_independent() {
        let mut com: ComManager<4, 16> = ComManager::new();
        com.add_pdu(tx_pdu(0x100, 10), &[u8_signal(1, 0, 0xAA)]);
        com.add_pdu(tx_pdu(0x200, 20), &[u8_signal(2, 0, 0xBB)]);
        assert_eq!(com.read_signal(1), Some(SignalValue::U8(0xAA)));
        assert_eq!(com.read_signal(2), Some(SignalValue::U8(0xBB)));

        com.write_signal(1, SignalValue::U8(0x11));
        assert_eq!(com.read_signal(1), Some(SignalValue::U8(0x11)));
        assert_eq!(com.read_signal(2), Some(SignalValue::U8(0xBB))); // unchanged
    }
}
