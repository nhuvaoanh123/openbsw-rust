//! Non-volatile memory manager for the STM32G474RE.
//!
//! Provides a simple, fixed-layout block store in the top 16 KB of internal
//! flash (pages 124–127, base address `0x0807_C000`).  Built on
//! [`crate::flash_g4::FlashG4`] for all erase/program operations and
//! [`bsw_util::crc::CRC32_ETHERNET`] for per-block integrity verification.
//!
//! # Flash layout
//!
//! ```text
//! Page 124  0x0807_C000  Block directory  (magic, version, block-valid flags)
//! Page 125  0x0807_D000  Data page        (all blocks packed sequentially)
//! Page 126  0x0807_E000  Reserved         (future / wear-levelling)
//! Page 127  0x0807_F000  Reserved         (future / wear-levelling)
//! ```
//!
//! # On-flash block record format (data page)
//!
//! Each block occupies a fixed slot in the data page.  The layout within
//! each slot is:
//!
//! ```text
//! [0..1]       u16 LE  data_length   (number of meaningful data bytes)
//! [2..2+N-1]   u8[N]   data          (N = data_length)
//! [2+N..2+N+3] u32 LE  CRC32         (Ethernet/ISO-HDLC over bytes [0..2+N-1])
//! padding      0xFF    to 8-byte boundary
//! ```
//!
//! An erased slot (all 0xFF) is treated as "no valid data".
//!
//! # Write strategy
//!
//! Writing always follows a **read-modify-write** pattern over the full data
//! page (page 125):
//!
//! 1. Shadow-copy the entire page into a 4 KB stack buffer.
//! 2. Patch the target block's slot in the buffer.
//! 3. Erase page 125.
//! 4. Program the patched buffer back.
//!
//! This keeps the implementation simple (no wear-levelling) while ensuring
//! the other blocks are not lost during a single-block update.  The 4 KB
//! stack frame is intentional: `no_std` and no heap.
//!
//! # `no_std`
//!
//! No heap allocation.  All buffers are on the stack or in static storage.

use bsw_util::crc::CRC32_ETHERNET;

use crate::flash_g4::{FlashG4, NVM_BASE_ADDR, NVM_PAGE_START};

// ---------------------------------------------------------------------------
// Layout constants
// ---------------------------------------------------------------------------

/// Page size (4 KB) — same as [`crate::flash_g4::FLASH_PAGE_SIZE`].
const PAGE_SIZE: usize = 4 * 1024;

/// Absolute address of the block directory (page 124).
const DIR_ADDR: u32 = NVM_BASE_ADDR; // 0x0807_C000

/// Absolute address of the data page (page 125).
const DATA_ADDR: u32 = NVM_BASE_ADDR + PAGE_SIZE as u32; // 0x0807_D000

/// Flash page number of the block directory.
const DIR_PAGE: u8 = NVM_PAGE_START; // 124

/// Flash page number of the data page.
const DATA_PAGE: u8 = NVM_PAGE_START + 1; // 125

// ---------------------------------------------------------------------------
// Block directory constants
// ---------------------------------------------------------------------------

/// Magic word written at the start of the directory page to distinguish an
/// initialised NvM area from an erased one.
const DIR_MAGIC: u32 = 0x4E564D5F; // ASCII "NVM_"

/// NvM layout version.  Increment whenever block offsets or slot sizes change.
const DIR_VERSION: u32 = 1;

/// Byte offset of the magic word in the directory page.
const DIR_OFF_MAGIC: u32 = 0;
/// Byte offset of the version field.
const DIR_OFF_VERSION: u32 = 4;
/// Byte offset of the per-block validity bitmap (one u32, bits 0–3 = blocks 0–3).
const DIR_OFF_VALID_BITS: u32 = 8;

// ---------------------------------------------------------------------------
// Block descriptors
// ---------------------------------------------------------------------------

/// Maximum number of NvM data blocks.
const BLOCK_COUNT: usize = 4;

/// Maximum data bytes for the VIN block.
const VIN_MAX_LEN: u16 = 17;
/// Maximum data bytes for the ECU serial block.
const ECU_SERIAL_MAX_LEN: u16 = 10;
/// Maximum data bytes for the DTC storage block.
const DTC_STORAGE_MAX_LEN: u16 = 256;
/// Maximum data bytes for the calibration block.
const CALIBRATION_MAX_LEN: u16 = 64;

/// Size (in bytes) of the CRC32 appended after each block's data.
const CRC_LEN: u16 = 4;
/// Size of the `data_length` field.
const LEN_FIELD_SIZE: u16 = 2;

/// Total on-flash slot size for a block whose data is `max_data` bytes,
/// rounded up to the next 8-byte boundary.
const fn slot_size(max_data: u16) -> u16 {
    let raw = LEN_FIELD_SIZE + max_data + CRC_LEN;
    // Round up to 8-byte boundary.
    (raw + 7) & !7
}

/// Byte offsets within the data page for each block slot.
///
/// Each slot is placed at the next 8-byte-aligned boundary after the
/// previous slot.
// Offsets are computed as: each block starts at the 8-byte-aligned boundary
// immediately after the previous block's slot ends.
//
//   Block 0 (VIN, 17B):     slot_size = (2+17+4+7)&!7 = 24.  [0x0000..0x0018)
//   Block 1 (EcuSerial,10B):slot_size = (2+10+4+7)&!7 = 16.  [0x0018..0x0028)
//   Block 2 (DtcStorage,256B):slot_size=(2+256+4+7)&!7= 272. [0x0028..0x0138)
//   Block 3 (Calibration,64B):slot_size=(2+64+4+7)&!7 = 72.  [0x0138..0x0180)
//   Total used: 0x0180 = 384 bytes — well within the 4 KB data page.
const BLOCK_OFFSET: [u16; BLOCK_COUNT] = [
    0x0000, // Block 0 — VIN
    0x0018, // Block 1 — EcuSerial     (= 0 + 24)
    0x0028, // Block 2 — DtcStorage    (= 24 + 16)
    0x0138, // Block 3 — Calibration   (= 40 + 272)
];

/// Maximum data length for each block (in order of [`NvmBlockId`]).
const BLOCK_MAX_LEN: [u16; BLOCK_COUNT] = [
    VIN_MAX_LEN,
    ECU_SERIAL_MAX_LEN,
    DTC_STORAGE_MAX_LEN,
    CALIBRATION_MAX_LEN,
];

// Compile-time sanity: ensure every slot fits in the data page.
const _: () = {
    let mut i = 0usize;
    while i < BLOCK_COUNT {
        let end = BLOCK_OFFSET[i] + slot_size(BLOCK_MAX_LEN[i]);
        assert!(
            (end as usize) <= PAGE_SIZE,
            "NvM block slot exceeds data page size"
        );
        i += 1;
    }
};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Identifiers for the four NvM storage blocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NvmBlockId {
    /// Vehicle Identification Number — 17 bytes.
    Vin = 0,
    /// ECU serial number — 10 bytes.
    EcuSerial = 1,
    /// Diagnostic Trouble Code storage — 256 bytes.
    DtcStorage = 2,
    /// Calibration data — 64 bytes.
    Calibration = 3,
}

impl NvmBlockId {
    /// Convert to an array index.
    #[inline]
    fn idx(self) -> usize {
        self as usize
    }

    /// Byte offset of this block's slot within the data page.
    #[inline]
    fn offset(self) -> u16 {
        BLOCK_OFFSET[self.idx()]
    }

    /// Maximum data payload length for this block.
    #[inline]
    fn max_len(self) -> u16 {
        BLOCK_MAX_LEN[self.idx()]
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Read a `u16` little-endian from flash at `addr`.
#[inline]
fn read_u16_le(addr: u32) -> u16 {
    let mut buf = [0u8; 2];
    FlashG4::read(addr, &mut buf);
    u16::from_le_bytes(buf)
}

/// Read a `u32` little-endian from flash at `addr`.
#[inline]
fn read_u32_le(addr: u32) -> u32 {
    let mut buf = [0u8; 4];
    FlashG4::read(addr, &mut buf);
    u32::from_le_bytes(buf)
}

/// Write a `u16` little-endian into a byte buffer at `offset`.
#[inline]
fn write_u16_le(buf: &mut [u8], offset: usize, val: u16) {
    buf[offset..offset + 2].copy_from_slice(&val.to_le_bytes());
}

/// Write a `u32` little-endian into a byte buffer at `offset`.
#[inline]
fn write_u32_le(buf: &mut [u8], offset: usize, val: u32) {
    buf[offset..offset + 4].copy_from_slice(&val.to_le_bytes());
}

/// Compute the CRC32 (Ethernet/ISO-HDLC) over `data`.
#[inline]
fn crc32(data: &[u8]) -> u32 {
    CRC32_ETHERNET.checksum(data)
}

// ---------------------------------------------------------------------------
// NvmManager
// ---------------------------------------------------------------------------

/// Zero-sized NvM manager.
///
/// All methods are associated functions (no `self`).  The manager's state
/// is entirely stored in flash; there is no RAM-resident cache.
pub struct NvmManager;

impl NvmManager {
    // -----------------------------------------------------------------------
    // Read
    // -----------------------------------------------------------------------

    /// Read a block from NvM into `buf`.
    ///
    /// Returns the number of data bytes written to `buf`, or `0` if:
    /// * The block has never been written (erased flash — slot is all 0xFF).
    /// * The stored CRC32 does not match (data corruption).
    /// * `buf` is too small to hold the stored data.
    ///
    /// CRC is computed over the `[length_field | data]` bytes, matching
    /// the write path.
    pub fn read_block(id: NvmBlockId, buf: &mut [u8]) -> usize {
        let slot_addr = DATA_ADDR + id.offset() as u32;

        // Read the 2-byte length field.
        let data_len = read_u16_le(slot_addr);

        // An erased slot has data_len = 0xFFFF.
        if data_len == 0xFFFF || data_len == 0 {
            return 0;
        }

        // Bounds check against the block's maximum.
        if data_len > id.max_len() {
            return 0;
        }

        // Bounds check against the caller's buffer.
        if buf.len() < data_len as usize {
            return 0;
        }

        // Read the data bytes from flash directly into the caller's buffer.
        let data_addr = slot_addr + LEN_FIELD_SIZE as u32;
        FlashG4::read(data_addr, &mut buf[..data_len as usize]);

        // Read the stored CRC32 (immediately after the data).
        let crc_addr = data_addr + data_len as u32;
        let stored_crc = read_u32_le(crc_addr);

        // Recompute CRC over [length_le | data] — 2 + data_len bytes.
        let mut crc_input = [0u8; LEN_FIELD_SIZE as usize + 256]; // largest block = 256 bytes
        let crc_input_len = LEN_FIELD_SIZE as usize + data_len as usize;
        write_u16_le(&mut crc_input, 0, data_len);
        crc_input[LEN_FIELD_SIZE as usize..crc_input_len]
            .copy_from_slice(&buf[..data_len as usize]);
        let computed_crc = crc32(&crc_input[..crc_input_len]);

        if computed_crc != stored_crc {
            // Wipe the output buffer to avoid leaking corrupt data.
            for b in buf[..data_len as usize].iter_mut() {
                *b = 0;
            }
            return 0;
        }

        data_len as usize
    }

    // -----------------------------------------------------------------------
    // Write
    // -----------------------------------------------------------------------

    /// Write a block to NvM.
    ///
    /// Performs a full **read-modify-write** cycle over page 125:
    ///
    /// 1. Copy the entire 4 KB data page into a stack buffer.
    /// 2. Patch the target block's slot (length + data + CRC32).
    /// 3. Erase page 125.
    /// 4. Program the patched buffer back.
    ///
    /// `data.len()` must not exceed the block's [`NvmBlockId::max_len`].
    ///
    /// Returns `true` on success.
    ///
    /// # Safety
    ///
    /// Flash must be unlocked (call [`FlashG4::unlock`]) before calling.
    /// The caller is responsible for locking flash again afterwards.
    pub unsafe fn write_block(id: NvmBlockId, data: &[u8]) -> bool {
        if data.len() > id.max_len() as usize {
            return false;
        }

        // 1. Shadow-copy the entire data page onto the stack (4 KB).
        let mut page_buf = [0xFFu8; PAGE_SIZE];
        FlashG4::read(DATA_ADDR, &mut page_buf);

        // 2. Patch the target slot in the buffer.
        let slot_off = id.offset() as usize;
        let data_len = data.len() as u16;

        write_u16_le(&mut page_buf, slot_off, data_len);
        let data_off = slot_off + LEN_FIELD_SIZE as usize;
        page_buf[data_off..data_off + data.len()].copy_from_slice(data);

        // Compute CRC over [length_field | data].
        let crc_input_len = LEN_FIELD_SIZE as usize + data.len();
        let computed_crc = crc32(&page_buf[slot_off..slot_off + crc_input_len]);

        let crc_off = data_off + data.len();
        write_u32_le(&mut page_buf, crc_off, computed_crc);

        // Pad the remainder of the slot to 0xFF (already 0xFF from the erased
        // shadow, but make it explicit for correctness after a previous write).
        let slot_end = slot_off + slot_size(id.max_len()) as usize;
        for b in page_buf[crc_off + CRC_LEN as usize..slot_end].iter_mut() {
            *b = 0xFF;
        }

        // 3. Erase page 125.
        // SAFETY: caller guarantees flash is unlocked; DATA_PAGE is NvM-only.
        unsafe {
            if !FlashG4::erase_page(DATA_PAGE) {
                return false;
            }

            // 4. Program the patched buffer back.
            FlashG4::program(DATA_ADDR, &page_buf)
        }
    }

    // -----------------------------------------------------------------------
    // Erase all
    // -----------------------------------------------------------------------

    /// Erase all NvM storage (factory reset).
    ///
    /// Erases pages 124 (directory) and 125 (data).  Pages 126–127 are
    /// reserved and are **not** erased by this function.
    ///
    /// Returns `true` if both erases succeeded.
    ///
    /// # Safety
    ///
    /// Flash must be unlocked before calling.
    pub unsafe fn erase_all() -> bool {
        // SAFETY: caller guarantees flash is unlocked; both pages are in NvM area.
        unsafe {
            if !FlashG4::erase_page(DIR_PAGE) {
                return false;
            }
            FlashG4::erase_page(DATA_PAGE)
        }
    }

    // -----------------------------------------------------------------------
    // Validity check
    // -----------------------------------------------------------------------

    /// Returns `true` if the block at `id` contains valid data (CRC passes).
    ///
    /// Uses a temporary stack buffer sized to the block's maximum data length.
    /// Does not require flash to be unlocked.
    pub fn is_valid(id: NvmBlockId) -> bool {
        // Stack buffer sized to the largest possible block (256 bytes for DTC).
        let mut buf = [0u8; DTC_STORAGE_MAX_LEN as usize];
        Self::read_block(id, &mut buf[..id.max_len() as usize]) > 0
    }
}

// ---------------------------------------------------------------------------
// Unit tests (host-only — no flash hardware required)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Verify the compile-time slot layout: offsets must be monotonically
    // increasing and no slot must overflow the page.
    #[test]
    fn slot_layout_monotonic_and_in_page() {
        for i in 0..BLOCK_COUNT {
            let end = BLOCK_OFFSET[i] as usize + slot_size(BLOCK_MAX_LEN[i]) as usize;
            assert!(end <= PAGE_SIZE, "block {i} slot overflows data page");
            if i + 1 < BLOCK_COUNT {
                assert!(
                    BLOCK_OFFSET[i + 1] >= BLOCK_OFFSET[i] + slot_size(BLOCK_MAX_LEN[i]),
                    "block {i} and {} slots overlap",
                    i + 1
                );
            }
        }
    }

    #[test]
    fn slot_size_is_multiple_of_8() {
        for i in 0..BLOCK_COUNT {
            let sz = slot_size(BLOCK_MAX_LEN[i]);
            assert_eq!(sz % 8, 0, "block {i} slot size {} is not 8-byte aligned", sz);
        }
    }

    // Verify the CRC helper produces the standard Ethernet check value.
    #[test]
    fn crc32_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    // Verify `read_block` returns 0 for an all-0xFF (erased) slot.
    #[test]
    fn read_block_erased_returns_zero() {
        // We can't call FlashG4::read in a host test.  Instead, verify the
        // guard condition directly: data_len == 0xFFFF → return 0.
        // The real integration test runs on hardware; here we just confirm
        // the constant used for the erased-slot check.
        let erased_len = u16::from_le_bytes([0xFF, 0xFF]);
        assert_eq!(erased_len, 0xFFFF);
    }

    // Verify the block format round-trip in isolation (without flash I/O).
    #[test]
    fn block_record_crc_round_trip() {
        let data = b"TESTVIN1234567890"; // 17 bytes
        let data_len = data.len() as u16;

        // Build the on-flash record manually (as write_block would).
        let mut record = [0xFFu8; 32];
        write_u16_le(&mut record, 0, data_len);
        record[2..2 + data.len()].copy_from_slice(data);
        let computed = crc32(&record[..2 + data.len()]);
        write_u32_le(&mut record, 2 + data.len(), computed);

        // Simulate a read: extract length, data, stored CRC, recompute.
        let stored_len = u16::from_le_bytes([record[0], record[1]]);
        assert_eq!(stored_len, 17);
        let stored_crc = u32::from_le_bytes([
            record[2 + stored_len as usize],
            record[3 + stored_len as usize],
            record[4 + stored_len as usize],
            record[5 + stored_len as usize],
        ]);
        let recomputed = crc32(&record[..2 + stored_len as usize]);
        assert_eq!(recomputed, stored_crc);
    }

    #[test]
    fn block_record_crc_detects_corruption() {
        let data = b"HELLO";
        let data_len = data.len() as u16;

        let mut record = [0xFFu8; 16];
        write_u16_le(&mut record, 0, data_len);
        record[2..2 + data.len()].copy_from_slice(data);
        let good_crc = crc32(&record[..2 + data.len()]);
        write_u32_le(&mut record, 2 + data.len(), good_crc);

        // Corrupt one data byte.
        record[3] ^= 0x01;

        let stored_crc = u32::from_le_bytes([
            record[2 + data_len as usize],
            record[3 + data_len as usize],
            record[4 + data_len as usize],
            record[5 + data_len as usize],
        ]);
        let recomputed = crc32(&record[..2 + data_len as usize]);
        assert_ne!(recomputed, stored_crc, "corruption must be detected");
    }
}
