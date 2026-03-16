// Copyright 2024 Accenture / Taktflow Systems.
// SPDX-License-Identifier: Apache-2.0

//! Signal types — AUTOSAR `ComSignalType` mapping and runtime signal values.
//!
//! A [`SignalDescriptor`] is a compile-time-constant struct that describes
//! where a signal lives inside a PDU (bit offset, width, byte order).
//! A [`SignalValue`] is the runtime value that flows between the application
//! and the COM layer at run-time.

// ---------------------------------------------------------------------------
// SignalType
// ---------------------------------------------------------------------------

/// Signal data type — matches AUTOSAR `ComSignalType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalType {
    /// 1-bit boolean.
    Boolean,
    /// 8-bit unsigned integer.
    Uint8,
    /// 16-bit unsigned integer.
    Uint16,
    /// 32-bit unsigned integer.
    Uint32,
    /// 8-bit signed integer.
    Sint8,
    /// 16-bit signed integer.
    Sint16,
    /// 32-bit signed integer.
    Sint32,
    /// Byte array of `N` bytes (AUTOSAR `UINT8_N`).
    Uint8N(usize),
}

// ---------------------------------------------------------------------------
// ByteOrder
// ---------------------------------------------------------------------------

/// Signal byte order inside a PDU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ByteOrder {
    /// Intel / little-endian: LSBit at `bit_position`.
    LittleEndian,
    /// Motorola / big-endian: MSBit at `bit_position` (DBC convention).
    BigEndian,
}

// ---------------------------------------------------------------------------
// SignalDescriptor
// ---------------------------------------------------------------------------

/// Static signal descriptor — defines where a signal lives in a PDU.
///
/// All fields are `pub` to allow `const` table initialisation in application
/// code without needing constructors.
#[derive(Debug, Clone, Copy)]
pub struct SignalDescriptor {
    /// Signal ID — unique within the system.
    pub id: u16,
    /// Bit position in the PDU (LSBit for LE, MSBit for BE).
    pub bit_position: u16,
    /// Width of the signal in bits.
    pub bit_size: u8,
    /// Data type.
    pub signal_type: SignalType,
    /// Byte order.
    pub byte_order: ByteOrder,
    /// Initial / default value packed as `u32`.
    pub init_value: u32,
}

// ---------------------------------------------------------------------------
// SignalValue
// ---------------------------------------------------------------------------

/// Runtime signal value — typed union of all supported primitive types.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SignalValue {
    /// Boolean signal.
    Bool(bool),
    /// 8-bit unsigned.
    U8(u8),
    /// 16-bit unsigned.
    U16(u16),
    /// 32-bit unsigned.
    U32(u32),
    /// 8-bit signed.
    I8(i8),
    /// 16-bit signed.
    I16(i16),
    /// 32-bit signed.
    I32(i32),
}

impl SignalValue {
    /// Losslessly encode any variant as a `u32` bit pattern.
    ///
    /// Signed values are zero-extended after casting to the corresponding
    /// unsigned type (same as how they are packed into the PDU buffer).
    #[must_use]
    pub fn as_u32(&self) -> u32 {
        match *self {
            Self::Bool(b) => u32::from(b),
            Self::U8(v) => u32::from(v),
            Self::U16(v) => u32::from(v),
            Self::U32(v) => v,
            Self::I8(v) => u32::from(v as u8),
            Self::I16(v) => u32::from(v as u16),
            Self::I32(v) => v as u32,
        }
    }

    /// Reconstruct a [`SignalValue`] from a `u32` bit pattern, guided by
    /// `sig_type`.
    ///
    /// For `Uint8N` arrays this returns `U32` with the raw bytes — callers
    /// that need the full array should handle the byte-array path separately.
    #[must_use]
    pub fn from_u32(val: u32, sig_type: SignalType) -> Self {
        match sig_type {
            SignalType::Boolean => Self::Bool(val != 0),
            SignalType::Uint8 => Self::U8(val as u8),
            SignalType::Uint16 => Self::U16(val as u16),
            SignalType::Uint32 => Self::U32(val),
            SignalType::Sint8 => Self::I8(val as i8),
            SignalType::Sint16 => Self::I16(val as i16),
            SignalType::Sint32 => Self::I32(val as i32),
            // For byte arrays, return the raw u32 — packer handles full copy.
            SignalType::Uint8N(_) => Self::U32(val),
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::{SignalType, SignalValue};

    #[test]
    fn bool_round_trip() {
        assert_eq!(SignalValue::Bool(true).as_u32(), 1);
        assert_eq!(SignalValue::Bool(false).as_u32(), 0);
        assert_eq!(SignalValue::from_u32(1, SignalType::Boolean), SignalValue::Bool(true));
        assert_eq!(SignalValue::from_u32(0, SignalType::Boolean), SignalValue::Bool(false));
    }

    #[test]
    fn u8_round_trip() {
        assert_eq!(SignalValue::U8(0xAB).as_u32(), 0xAB);
        assert_eq!(SignalValue::from_u32(0xAB, SignalType::Uint8), SignalValue::U8(0xAB));
    }

    #[test]
    fn u16_round_trip() {
        assert_eq!(SignalValue::U16(0xBEEF).as_u32(), 0xBEEF);
        assert_eq!(SignalValue::from_u32(0xBEEF, SignalType::Uint16), SignalValue::U16(0xBEEF));
    }

    #[test]
    fn u32_round_trip() {
        assert_eq!(SignalValue::U32(0xDEAD_BEEF).as_u32(), 0xDEAD_BEEF);
        assert_eq!(
            SignalValue::from_u32(0xDEAD_BEEF, SignalType::Uint32),
            SignalValue::U32(0xDEAD_BEEF)
        );
    }

    #[test]
    fn i8_negative_round_trip() {
        let v = SignalValue::I8(-1_i8);
        let raw = v.as_u32();
        assert_eq!(raw, 0xFF);
        assert_eq!(SignalValue::from_u32(raw, SignalType::Sint8), SignalValue::I8(-1));
    }

    #[test]
    fn i16_negative_round_trip() {
        let v = SignalValue::I16(-256_i16);
        let raw = v.as_u32();
        assert_eq!(raw, 0xFF00);
        assert_eq!(SignalValue::from_u32(raw, SignalType::Sint16), SignalValue::I16(-256));
    }

    #[test]
    fn i32_negative_round_trip() {
        let v = SignalValue::I32(-1_i32);
        let raw = v.as_u32();
        assert_eq!(raw, 0xFFFF_FFFF);
        assert_eq!(SignalValue::from_u32(raw, SignalType::Sint32), SignalValue::I32(-1));
    }
}
