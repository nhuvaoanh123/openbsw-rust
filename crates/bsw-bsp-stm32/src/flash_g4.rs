//! STM32G474RE internal flash controller driver.
//!
//! Implements page erase and doubleword programming for the G4 dual-bank
//! flash controller (RM0440 §3).  We operate exclusively in **single-bank**
//! mode (512 KB, 128 pages × 4 KB).
//!
//! # NvM reservation
//!
//! Pages 124–127 (the top 16 KB) are reserved for non-volatile storage.
//! See [`NVM_BASE_ADDR`], [`NVM_SIZE`], [`NVM_PAGE_START`], [`NVM_PAGE_COUNT`].
//!
//! # Safety
//!
//! All mutating functions are `unsafe` because:
//! * They write to memory-mapped registers with side-effects.
//! * Flash programming while running from flash is only safe when the caller
//!   guarantees the CPU is not fetching instructions from the page being
//!   erased/written (or the entire NvM area, which lives at the top of flash
//!   and is never executed).
//! * They must not be called from within a hard-fault or NMI handler.

// ---------------------------------------------------------------------------
// Register map  (FLASH base = 0x4002_2000, RM0440 Table 7)
// ---------------------------------------------------------------------------

const FLASH_BASE: u32 = 0x4002_2000;

/// Access-control register (cache enable, wait states — already set by clock_g4).
const FLASH_ACR: *mut u32 = (FLASH_BASE + 0x000) as *mut u32;
/// Key register — write unlock sequence to clear the LOCK bit in CR.
const FLASH_KEYR: *mut u32 = (FLASH_BASE + 0x008) as *mut u32;
/// Status register.
const FLASH_SR: *mut u32 = (FLASH_BASE + 0x010) as *mut u32;
/// Control register.
const FLASH_CR: *mut u32 = (FLASH_BASE + 0x014) as *mut u32;

// --- FLASH_SR bit positions ---
/// End-of-operation flag (write 1 to clear).
const SR_EOP: u32 = 1 << 0;
/// Programming error flags (bits 1–8 inclusive).
const SR_ERROR_MASK: u32 = 0x0000_01FE;
/// Busy flag — set while a flash operation is in progress.
const SR_BSY: u32 = 1 << 16;

// --- FLASH_CR bit positions ---
/// Programming enable.
const CR_PG: u32 = 1 << 0;
/// Page-erase enable.
const CR_PER: u32 = 1 << 1;
/// Page-number field: bits [10:3] (8-bit value, zero-based page index).
const CR_PNB_SHIFT: u32 = 3;
const CR_PNB_MASK: u32 = 0xFF << CR_PNB_SHIFT;
/// Start erase operation.
const CR_STRT: u32 = 1 << 16;
/// Lock bit — set to re-enable write protection, clear via unlock sequence.
const CR_LOCK: u32 = 1 << 31;

// --- Unlock keys (RM0440 §3.3.5) ---
const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

// ---------------------------------------------------------------------------
// Flash address constants
// ---------------------------------------------------------------------------

/// First valid flash address.
pub const FLASH_BASE_ADDR: u32 = 0x0800_0000;
/// Total flash size: 512 KB.
pub const FLASH_SIZE: u32 = 512 * 1024;
/// Number of pages.
pub const FLASH_PAGE_COUNT: u8 = 128;
/// Page size: 4 KB.
pub const FLASH_PAGE_SIZE: u32 = 4 * 1024;
/// Program granularity: 8 bytes (doubleword).
pub const FLASH_PROG_GRANULARITY: u32 = 8;

// ---------------------------------------------------------------------------
// NvM storage area (pages 124–127, top 16 KB)
// ---------------------------------------------------------------------------

/// Base address of the NvM storage area (page 124).
pub const NVM_BASE_ADDR: u32 = 0x0807_C000;
/// Total size of the NvM storage area.
pub const NVM_SIZE: u32 = 16 * 1024;
/// First NvM page number.
pub const NVM_PAGE_START: u8 = 124;
/// Number of NvM pages.
pub const NVM_PAGE_COUNT: u8 = 4;

// ---------------------------------------------------------------------------
// Helper: volatile register access
// ---------------------------------------------------------------------------

#[inline(always)]
unsafe fn read_reg(reg: *const u32) -> u32 {
    // SAFETY: caller guarantees the pointer is a valid MMIO address.
    unsafe { core::ptr::read_volatile(reg) }
}

#[inline(always)]
unsafe fn write_reg(reg: *mut u32, val: u32) {
    // SAFETY: caller guarantees the pointer is a valid MMIO address.
    unsafe { core::ptr::write_volatile(reg, val) }
}

#[inline(always)]
unsafe fn modify_reg<F: FnOnce(u32) -> u32>(reg: *mut u32, f: F) {
    // SAFETY: caller guarantees the pointer is a valid MMIO address.
    unsafe {
        let v = read_reg(reg as *const u32);
        write_reg(reg, f(v));
    }
}

// ---------------------------------------------------------------------------
// FlashG4
// ---------------------------------------------------------------------------

/// Zero-sized driver for the STM32G474RE embedded flash controller.
///
/// All methods are associated functions (no `self`) because the peripheral
/// is a singleton accessed through memory-mapped registers.
pub struct FlashG4;

impl FlashG4 {
    // -----------------------------------------------------------------------
    // Lock / unlock
    // -----------------------------------------------------------------------

    /// Unlock flash for programming and erase operations.
    ///
    /// Returns `true` if the flash was successfully unlocked (LOCK bit is now
    /// clear), `false` if it was already unlocked (no-op — not an error in
    /// practice, but the caller should be aware).
    ///
    /// # Safety
    ///
    /// Must not be called concurrently or from an interrupt that could
    /// preempt another flash operation.
    pub unsafe fn unlock() -> bool {
        // SAFETY: FLASH_CR and FLASH_KEYR are valid MMIO addresses.
        unsafe {
            let cr = read_reg(FLASH_CR as *const u32);
            if cr & CR_LOCK == 0 {
                // Already unlocked.
                return false;
            }
            // Write the two-key unlock sequence.
            write_reg(FLASH_KEYR, KEY1);
            write_reg(FLASH_KEYR, KEY2);
            // Verify the LOCK bit was cleared.
            read_reg(FLASH_CR as *const u32) & CR_LOCK == 0
        }
    }

    /// Lock flash, re-enabling hardware write protection.
    ///
    /// # Safety
    ///
    /// Should only be called after all programming/erase operations are
    /// complete and the flash is not busy.
    pub unsafe fn lock() {
        // SAFETY: FLASH_CR is a valid MMIO address.
        unsafe {
            modify_reg(FLASH_CR, |v| v | CR_LOCK);
        }
    }

    // -----------------------------------------------------------------------
    // Status helpers
    // -----------------------------------------------------------------------

    /// Returns `true` while a flash operation (erase or program) is in progress.
    #[inline]
    pub fn is_busy() -> bool {
        // SAFETY: FLASH_SR is a valid read-only MMIO address for status polling.
        unsafe { read_reg(FLASH_SR as *const u32) & SR_BSY != 0 }
    }

    /// Spin-wait until the flash controller is idle.
    ///
    /// Returns `true` on success, `false` if any error bit was set in `FLASH_SR`
    /// at the end of the operation.
    ///
    /// # Safety
    ///
    /// Must be called only when a flash operation has been started (after
    /// setting CR_STRT or CR_PG).  Calling this with no operation in
    /// progress is harmless but wasteful.
    pub unsafe fn wait_ready() -> bool {
        // SAFETY: polling FLASH_SR is safe (read-only status register).
        unsafe {
            while Self::is_busy() {
                // Insert a compiler fence to prevent the optimizer from hoisting
                // the load out of the loop.
                core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
            }
            // Return false if any error flag is set.
            read_reg(FLASH_SR as *const u32) & SR_ERROR_MASK == 0
        }
    }

    /// Clear all error flags in `FLASH_SR` (write-1-to-clear, RM0440 §3.6.5).
    ///
    /// # Safety
    ///
    /// Must only be called when the flash is not busy (BSY = 0).
    pub unsafe fn clear_errors() {
        // SAFETY: writing W1C bits to FLASH_SR.
        unsafe {
            write_reg(FLASH_SR, SR_ERROR_MASK | SR_EOP);
        }
    }

    // -----------------------------------------------------------------------
    // Erase
    // -----------------------------------------------------------------------

    /// Erase a single 4 KB page.
    ///
    /// `page_num` must be in the range `0..128`.  The caller is responsible
    /// for ensuring the page is not currently being executed from.
    ///
    /// Returns `true` on success, `false` on invalid page or hardware error.
    ///
    /// # Erase sequence (RM0440 §3.3.7)
    ///
    /// 1. Verify BSY = 0.
    /// 2. Clear error flags.
    /// 3. Set `PER` in `CR`.
    /// 4. Write page number into `CR[PNB]`.
    /// 5. Set `STRT` in `CR`.
    /// 6. Wait for BSY = 0.
    /// 7. Check and clear `EOP`.
    /// 8. Clear `PER` in `CR`.
    ///
    /// # Safety
    ///
    /// Flash must be unlocked before calling.  Caller must guarantee the CPU
    /// is not fetching code from `page_num`.
    pub unsafe fn erase_page(page_num: u8) -> bool {
        if page_num >= FLASH_PAGE_COUNT {
            return false;
        }

        // SAFETY: all register accesses are to valid MMIO addresses.
        unsafe {
            // 1. Verify not busy.
            if Self::is_busy() {
                return false;
            }

            // 2. Clear errors.
            Self::clear_errors();

            // 3 + 4. Set PER and page number.
            modify_reg(FLASH_CR, |v| {
                let v = v & !CR_PNB_MASK; // clear existing PNB
                let v = v | ((page_num as u32) << CR_PNB_SHIFT);
                v | CR_PER
            });

            // 5. Set STRT to trigger the erase.
            modify_reg(FLASH_CR, |v| v | CR_STRT);

            // 6. Wait for completion.
            let ok = Self::wait_ready();

            // 7. Clear EOP.
            let sr = read_reg(FLASH_SR as *const u32);
            if sr & SR_EOP != 0 {
                write_reg(FLASH_SR, SR_EOP);
            }

            // 8. Clear PER.
            modify_reg(FLASH_CR, |v| v & !CR_PER);

            ok
        }
    }

    // -----------------------------------------------------------------------
    // Program
    // -----------------------------------------------------------------------

    /// Program a single 8-byte doubleword at `addr`.
    ///
    /// `addr` must be 8-byte aligned and within the flash address range
    /// (`0x0800_0000` .. `0x0807_FFFF`).
    ///
    /// The data is supplied as a `u64` (little-endian byte order on Cortex-M).
    /// Internally, two consecutive 32-bit writes are performed as required by
    /// the hardware.
    ///
    /// Returns `true` on success.
    ///
    /// # Program sequence (RM0440 §3.3.6)
    ///
    /// 1. Verify BSY = 0.
    /// 2. Set `PG` in `CR`.
    /// 3. Write the lower 32 bits, then the upper 32 bits to `addr`.
    /// 4. Wait for BSY = 0.
    /// 5. Check and clear `EOP`.
    /// 6. Clear `PG` in `CR`.
    ///
    /// # Safety
    ///
    /// Flash must be unlocked.  `addr` must be 8-byte aligned and in range.
    /// Destination doubleword must have been erased (all 0xFF) beforehand.
    pub unsafe fn program_doubleword(addr: u32, data: u64) -> bool {
        // Validate address alignment and range.
        if addr & 0x7 != 0 {
            return false;
        }
        if addr < FLASH_BASE_ADDR || addr > FLASH_BASE_ADDR + FLASH_SIZE - 8 {
            return false;
        }

        // SAFETY: all register and flash address accesses are valid.
        unsafe {
            // 1. Verify not busy.
            if Self::is_busy() {
                return false;
            }

            // 2. Set PG.
            modify_reg(FLASH_CR, |v| v | CR_PG);

            // 3. Write the two 32-bit words (little-endian on Cortex-M).
            let lo = data as u32;
            let hi = (data >> 32) as u32;
            core::ptr::write_volatile(addr as *mut u32, lo);
            core::ptr::write_volatile((addr + 4) as *mut u32, hi);

            // 4. Wait for completion.
            let ok = Self::wait_ready();

            // 5. Clear EOP.
            let sr = read_reg(FLASH_SR as *const u32);
            if sr & SR_EOP != 0 {
                write_reg(FLASH_SR, SR_EOP);
            }

            // 6. Clear PG.
            modify_reg(FLASH_CR, |v| v & !CR_PG);

            ok
        }
    }

    /// Program a byte slice to flash, handling alignment internally.
    ///
    /// `addr` must be 8-byte aligned.  The slice is padded with `0xFF` bytes
    /// to the next 8-byte boundary before the last doubleword write, matching
    /// the erased (all-ones) state of unprogrammed flash.
    ///
    /// Returns `true` if all doubleword writes succeeded.
    ///
    /// # Safety
    ///
    /// Flash must be unlocked.  `addr` must be 8-byte aligned and the entire
    /// region `[addr, addr + data.len())` (rounded up to 8 bytes) must lie
    /// within the valid flash range and have been erased.
    pub unsafe fn program(addr: u32, data: &[u8]) -> bool {
        if addr & 0x7 != 0 {
            return false;
        }
        if data.is_empty() {
            return true;
        }

        let mut offset = 0usize;
        let mut current_addr = addr;

        // SAFETY: preconditions are verified above; individual doubleword
        // writes are delegated to `program_doubleword` which re-validates.
        unsafe {
            while offset < data.len() {
                // Assemble one 8-byte doubleword from up to 8 bytes of `data`,
                // padding any tail bytes with 0xFF.
                let mut dw_bytes = [0xFFu8; 8];
                let chunk_len = core::cmp::min(8, data.len() - offset);
                dw_bytes[..chunk_len].copy_from_slice(&data[offset..offset + chunk_len]);

                let dw = u64::from_le_bytes(dw_bytes);
                if !Self::program_doubleword(current_addr, dw) {
                    return false;
                }

                offset += chunk_len;
                current_addr += 8;
            }
        }

        true
    }

    // -----------------------------------------------------------------------
    // Read
    // -----------------------------------------------------------------------

    /// Read bytes from flash into `buf`.
    ///
    /// Flash is memory-mapped, so this is simply a `copy_nonoverlapping`.
    /// No unlock is required for reads.
    pub fn read(addr: u32, buf: &mut [u8]) {
        if buf.is_empty() {
            return;
        }
        // SAFETY: `addr` points into the read-only flash address space which
        // is always valid while the device is running.  The caller must
        // ensure `addr + buf.len()` stays within the 512 KB flash region.
        unsafe {
            core::ptr::copy_nonoverlapping(addr as *const u8, buf.as_mut_ptr(), buf.len());
        }
    }
}
