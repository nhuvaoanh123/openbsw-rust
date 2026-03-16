//! # bsw-com
//!
//! AUTOSAR-style COM layer — signal packing/unpacking for CAN PDUs.
//!
//! Provides the bridge between application-level signal values (typed, named)
//! and raw CAN frame bytes (packed PDUs).
//!
//! ## Architecture
//!
//! ```text
//! Application (SWCs)
//!     ↓ write_signal() / read_signal()
//! COM Layer  (this crate)
//!     ↓ pack into PDU / unpack from PDU
//! CAN Transceiver  (bsw-can)
//!     ↓ CanFrame::with_data()
//! Hardware
//! ```
//!
//! ## Quick start
//!
//! ```rust
//! use bsw_com::com::ComManager;
//! use bsw_com::pdu::{PduDescriptor, PduDirection};
//! use bsw_com::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};
//!
//! // 1. Create a manager with capacity for 4 PDUs and 16 signals.
//! let mut com: ComManager<4, 16> = ComManager::new();
//!
//! // 2. Register a cyclic TX PDU (10 ms period) with two signals.
//! let signals = [
//!     SignalDescriptor {
//!         id: 1, bit_position: 0, bit_size: 8,
//!         signal_type: SignalType::Uint8, byte_order: ByteOrder::LittleEndian,
//!         init_value: 0,
//!     },
//!     SignalDescriptor {
//!         id: 2, bit_position: 8, bit_size: 16,
//!         signal_type: SignalType::Uint16, byte_order: ByteOrder::LittleEndian,
//!         init_value: 0,
//!     },
//! ];
//! com.add_pdu(
//!     PduDescriptor {
//!         can_id: 0x123, length: 8, direction: PduDirection::Tx,
//!         cycle_time_ms: 10, signal_count: 0, signal_start_index: 0,
//!     },
//!     &signals,
//! );
//!
//! // 3. Application writes a signal.
//! com.write_signal(1, SignalValue::U8(0x42));
//! com.write_signal(2, SignalValue::U16(0x1234));
//!
//! // 4. Periodic tick — collect frames ready for TX.
//! for (can_id, data) in com.tick(10) {
//!     // hand (can_id, data) to the CAN transceiver …
//!     let _ = (can_id, data);
//! }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

pub mod com;
pub mod packer;
pub mod pdu;
pub mod signal;

pub use com::{ComManager, ComTxIterator};
pub use pdu::{PduDescriptor, PduDirection};
pub use signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};
