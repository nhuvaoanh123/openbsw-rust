// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Signal packing and unpacking into/from CAN PDU byte buffers.
//!
//! # v1 scope
//!
//! Only **byte-aligned** signals are supported in this version
//! (`bit_position % 8 == 0`, `bit_size % 8 == 0`).  This covers ≥ 95 % of
//! production automotive CAN databases.  Sub-byte / non-aligned signals can
//! be added in a future revision.
//!
//! # Byte-order handling
//!
//! - **Little-endian (Intel)**: bytes are laid out from `byte_start` upward
//!   in LSByte-first order.
//! - **Big-endian (Motorola)**: bytes are laid out from `byte_start` downward
//!   in MSByte-first order (DBC `bit_position` convention pointing at MSBit).
//!
//! For byte-aligned signals the two encodings differ only in the order of the
//! multi-byte words, so the same `byte_start` / `byte_len` arithmetic applies
//! and only the byte-copy direction changes.

use crate::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};

// ---------------------------------------------------------------------------
// pack_signal
// ---------------------------------------------------------------------------

/// Pack `value` into `buf` at the position described by `descriptor`.
///
/// # Panics
///
/// Panics if the signal extends beyond `buf`.
pub fn pack_signal(buf: &mut [u8], descriptor: &SignalDescriptor, value: SignalValue) {
    let byte_start = (descriptor.bit_position / 8) as usize;
    let byte_len = (descriptor.bit_size / 8).max(1) as usize;

    assert!(
        byte_start + byte_len <= buf.len(),
        "pack_signal: signal (start={byte_start}, len={byte_len}) exceeds buf len {}",
        buf.len()
    );

    // Special path: Uint8N copies raw bytes from value (stored as U32 for
    // single-byte and up to 4 bytes; larger arrays handled via U32 lower bytes).
    if let SignalType::Uint8N(n) = descriptor.signal_type {
        let raw = value.as_u32();
        let copy_len = n.min(4).min(byte_len);
        // Always write little-endian bytes from raw u32.
        for i in 0..copy_len {
            buf[byte_start + i] = (raw >> (i * 8)) as u8;
        }
        return;
    }

    let raw: u32 = value.as_u32();

    match descriptor.byte_order {
        ByteOrder::LittleEndian => {
            // LSByte at byte_start, MSByte at byte_start + byte_len - 1.
            for i in 0..byte_len {
                buf[byte_start + i] = (raw >> (i * 8)) as u8;
            }
        }
        ByteOrder::BigEndian => {
            // MSByte at byte_start, LSByte at byte_start + byte_len - 1.
            for i in 0..byte_len {
                buf[byte_start + i] = (raw >> ((byte_len - 1 - i) * 8)) as u8;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// unpack_signal
// ---------------------------------------------------------------------------

/// Unpack a signal value from `buf` at the position described by `descriptor`.
///
/// # Panics
///
/// Panics if the signal extends beyond `buf`.
pub fn unpack_signal(buf: &[u8], descriptor: &SignalDescriptor) -> SignalValue {
    let byte_start = (descriptor.bit_position / 8) as usize;
    let byte_len = (descriptor.bit_size / 8).max(1) as usize;

    assert!(
        byte_start + byte_len <= buf.len(),
        "unpack_signal: signal (start={byte_start}, len={byte_len}) exceeds buf len {}",
        buf.len()
    );

    // Special path for Uint8N — reconstruct up to 4 bytes into U32.
    if let SignalType::Uint8N(_) = descriptor.signal_type {
        let mut raw: u32 = 0;
        let copy_len = byte_len.min(4);
        for i in 0..copy_len {
            raw |= u32::from(buf[byte_start + i]) << (i * 8);
        }
        return SignalValue::U32(raw);
    }

    let raw: u32 = match descriptor.byte_order {
        ByteOrder::LittleEndian => {
            let mut v: u32 = 0;
            for i in 0..byte_len {
                v |= u32::from(buf[byte_start + i]) << (i * 8);
            }
            v
        }
        ByteOrder::BigEndian => {
            let mut v: u32 = 0;
            for i in 0..byte_len {
                v |= u32::from(buf[byte_start + i]) << ((byte_len - 1 - i) * 8);
            }
            v
        }
    };

    // Boolean is special: any non-zero bit pattern is `true`.
    if descriptor.signal_type == SignalType::Boolean {
        let mask: u32 = if descriptor.bit_size >= 32 {
            u32::MAX
        } else {
            (1u32 << descriptor.bit_size) - 1
        };
        return SignalValue::Bool((raw & mask) != 0);
    }

    SignalValue::from_u32(raw, descriptor.signal_type)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{pack_signal, unpack_signal};
    use crate::signal::{ByteOrder, SignalDescriptor, SignalType, SignalValue};

    fn make_desc(
        id: u16,
        bit_position: u16,
        bit_size: u8,
        sig_type: SignalType,
        byte_order: ByteOrder,
        init_value: u32,
    ) -> SignalDescriptor {
        SignalDescriptor { id, bit_position, bit_size, signal_type: sig_type, byte_order, init_value }
    }

    // 1 — pack/unpack u8 at byte boundary (LE)
    #[test]
    fn u8_at_byte_boundary_le() {
        let desc = make_desc(1, 8, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0xAB));
        assert_eq!(buf[1], 0xAB);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U8(0xAB));
    }

    // 2 — pack/unpack u16 little-endian
    #[test]
    fn u16_little_endian() {
        let desc = make_desc(2, 0, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0x1234));
        // LE: LSByte at buf[0], MSByte at buf[1]
        assert_eq!(buf[0], 0x34);
        assert_eq!(buf[1], 0x12);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U16(0x1234));
    }

    // 3 — pack/unpack u16 big-endian
    #[test]
    fn u16_big_endian() {
        let desc = make_desc(3, 0, 16, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0x1234));
        // BE: MSByte at buf[0], LSByte at buf[1]
        assert_eq!(buf[0], 0x12);
        assert_eq!(buf[1], 0x34);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U16(0x1234));
    }

    // 4 — pack/unpack u32 little-endian
    #[test]
    fn u32_little_endian() {
        let desc = make_desc(4, 0, 32, SignalType::Uint32, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U32(0xDEAD_BEEF));
        assert_eq!(&buf[0..4], &[0xEF, 0xBE, 0xAD, 0xDE]);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U32(0xDEAD_BEEF));
    }

    // 5 — pack/unpack boolean (1 byte wide in v1, byte-aligned)
    #[test]
    fn boolean_pack_unpack() {
        let desc = make_desc(5, 8, 8, SignalType::Boolean, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::Bool(true));
        assert_eq!(buf[1], 1);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::Bool(true));

        pack_signal(&mut buf, &desc, SignalValue::Bool(false));
        assert_eq!(buf[1], 0);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::Bool(false));
    }

    // 6 — multiple signals in one PDU do not overlap
    #[test]
    fn multiple_signals_no_overlap() {
        let desc_a = make_desc(10, 0, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let desc_b = make_desc(11, 8, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let desc_c = make_desc(12, 16, 16, SignalType::Uint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];

        pack_signal(&mut buf, &desc_a, SignalValue::U8(0x11));
        pack_signal(&mut buf, &desc_b, SignalValue::U8(0x22));
        pack_signal(&mut buf, &desc_c, SignalValue::U16(0x3344));

        assert_eq!(unpack_signal(&buf, &desc_a), SignalValue::U8(0x11));
        assert_eq!(unpack_signal(&buf, &desc_b), SignalValue::U8(0x22));
        assert_eq!(unpack_signal(&buf, &desc_c), SignalValue::U16(0x3344));
    }

    // 7 — signed i8 negative round-trip
    #[test]
    fn i8_negative_pack_unpack() {
        let desc = make_desc(20, 0, 8, SignalType::Sint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::I8(-42));
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::I8(-42));
    }

    // 8 — signed i16 negative round-trip
    #[test]
    fn i16_negative_pack_unpack() {
        let desc = make_desc(21, 0, 16, SignalType::Sint16, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::I16(-1000));
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::I16(-1000));
    }

    // 9 — pack/unpack at byte 7 (last byte of 8-byte PDU)
    #[test]
    fn u8_at_last_byte() {
        let desc = make_desc(30, 56, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0xFF));
        assert_eq!(buf[7], 0xFF);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U8(0xFF));
    }

    // 10 — pack does not corrupt adjacent bytes
    #[test]
    fn pack_does_not_corrupt_adjacent() {
        let desc = make_desc(40, 8, 8, SignalType::Uint8, ByteOrder::LittleEndian, 0);
        let mut buf = [0xAAu8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U8(0x55));
        assert_eq!(buf[0], 0xAA); // byte 0 untouched
        assert_eq!(buf[1], 0x55); // written
        assert_eq!(buf[2], 0xAA); // byte 2 untouched
    }

    // 11 — boolean non-zero raw → true
    #[test]
    fn boolean_nonzero_is_true() {
        let desc = make_desc(50, 0, 8, SignalType::Boolean, ByteOrder::LittleEndian, 0);
        let buf = [0x05, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::Bool(true));
    }

    // 12 — u32 big-endian round-trip
    #[test]
    fn u32_big_endian() {
        let desc = make_desc(60, 0, 32, SignalType::Uint32, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U32(0x0102_0304));
        assert_eq!(&buf[0..4], &[0x01, 0x02, 0x03, 0x04]);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U32(0x0102_0304));
    }

    // 13 — u16 BE at byte offset 4
    #[test]
    fn u16_be_offset() {
        let desc = make_desc(70, 32, 16, SignalType::Uint16, ByteOrder::BigEndian, 0);
        let mut buf = [0u8; 8];
        pack_signal(&mut buf, &desc, SignalValue::U16(0xABCD));
        assert_eq!(buf[4], 0xAB);
        assert_eq!(buf[5], 0xCD);
        assert_eq!(unpack_signal(&buf, &desc), SignalValue::U16(0xABCD));
    }
}
