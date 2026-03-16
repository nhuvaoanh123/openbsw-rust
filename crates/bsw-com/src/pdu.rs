// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! PDU (Protocol Data Unit) descriptor — maps a CAN frame to its signals.
//!
//! A [`PduDescriptor`] is a compile-time-constant struct that ties a CAN ID
//! to a set of signals, a direction (TX/RX), a payload length, and an optional
//! cyclic transmission period.

// ---------------------------------------------------------------------------
// PduDirection
// ---------------------------------------------------------------------------

/// Transmission direction of a PDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PduDirection {
    /// Transmitted by this node.
    Tx,
    /// Received from the bus.
    Rx,
}

// ---------------------------------------------------------------------------
// PduDescriptor
// ---------------------------------------------------------------------------

/// PDU descriptor — maps a CAN ID to its signals and timing parameters.
///
/// All fields are `pub` to allow `const` table initialisation in application
/// code without needing constructors.
#[derive(Debug, Clone, Copy)]
pub struct PduDescriptor {
    /// CAN ID for this PDU (11-bit base or 29-bit extended raw value).
    pub can_id: u32,
    /// PDU payload length in bytes (1–8 for classic CAN, 1–64 for CAN-FD).
    pub length: u8,
    /// TX or RX.
    pub direction: PduDirection,
    /// Cyclic TX period in milliseconds; `0` = event-triggered only.
    pub cycle_time_ms: u16,
    /// Number of signals that belong to this PDU.
    pub signal_count: u8,
    /// Start index into the global [`SignalDescriptor`] table held by
    /// [`ComManager`](crate::com::ComManager).
    pub signal_start_index: u16,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{PduDescriptor, PduDirection};

    #[test]
    fn pdu_descriptor_fields() {
        let pdu = PduDescriptor {
            can_id: 0x123,
            length: 8,
            direction: PduDirection::Tx,
            cycle_time_ms: 10,
            signal_count: 3,
            signal_start_index: 0,
        };
        assert_eq!(pdu.can_id, 0x123);
        assert_eq!(pdu.length, 8);
        assert_eq!(pdu.direction, PduDirection::Tx);
        assert_eq!(pdu.cycle_time_ms, 10);
    }

    #[test]
    fn rx_pdu_descriptor() {
        let pdu = PduDescriptor {
            can_id: 0x456,
            length: 4,
            direction: PduDirection::Rx,
            cycle_time_ms: 0,
            signal_count: 1,
            signal_start_index: 5,
        };
        assert_eq!(pdu.direction, PduDirection::Rx);
    }
}
