#![no_main]
#![no_std]

use stm32f1xx_hal::flash;

// global logger
#[cfg(feature = "defmt")]
mod feature_defmt {
    use core::sync::atomic;
    pub use {defmt_rtt as _, panic_probe as _};

    static COUNT: atomic::AtomicUsize = atomic::AtomicUsize::new(0);
    defmt::timestamp!(
        "{=usize}",
        COUNT.fetch_add(1, atomic::Ordering::Relaxed)
    );
}

/// If this value is found at the address 0x2000_0000 (beginning of RAM),
/// bootloader will enter DFU mode. See memory.x linker script.
pub const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;

/// Board flash configuration. MEM_INFO_STRING below must also be changed.
pub const FLASH_SIZE: flash::FlashSize = flash::FlashSize::Sz64K;
pub const FLASH_SIZE_BYTES: usize = (FLASH_SIZE as usize) * 1024;
pub const BOOTLOADER_SIZE_BYTES: u32 = 16 * 1024;
pub const FW_ADDRESS: u32 = 0x0800_4000;
