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

/// Board flash configuration. MEM_INFO_STRING below must also be changed.
pub const FLASH_SIZE: flash::FlashSize = flash::FlashSize::Sz64K; // 64KB blue pill flash
pub const FLASH_SIZE_BYTES: usize = (FLASH_SIZE as usize) * 1024;
pub const BOOTLOADER_SIZE_BYTES: u32 = 16 * 1024; // actual size is ~12-13KB
pub const FIRMWARE_ADDRESS: u32 = 0x0800_4000;

// USB VID/PID for the bootloader
// see https://github.com/pidcodes/pidcodes.github.com/pull/1027
pub const USB_VID: u16 = 0x1209;
pub const USB_PID: u16 = 0x2444;

pub struct DfuCtl;

impl DfuCtl {
    /// If this value is found at the address 0x2000_0000 (beginning of RAM),
    /// bootloader will enter DFU mode. See memory.x linker script.
    const KEY_STAY_IN_BOOT: u32 = 0xb0d42b89;

    // see memory.x linker script
    fn magic_mut_ptr() -> *mut u32 {
        unsafe extern "C" {
            #[link_name = "_dfu_magic"]
            static mut magic: u32;
        }
        core::ptr::addr_of_mut!(magic)
    }

    // The function reads the magic value from the magic address in memory.
    // The magic address is defined in the linker script as _dfu_magic.
    fn get_magic_val() -> u32 {
        unsafe { Self::magic_mut_ptr().read_volatile() }
    }

    // Valid Range Check: The function then checks if the stack pointer (sp)
    // is within a valid range by performing a bitwise AND operation with
    // 0xfffe_0000 and comparing the result to 0x2000_0000. This check ensures
    // that the stack pointer is correctly aligned and within the expected
    // memory region for the application.
    // Specifically:
    // The bitwise AND operation sp & 0xfffe_0000 masks the lower 17 bits of the
    // stack pointer, effectively aligning it to a 128 KB boundary.
    // The result is then compared to 0x2000_0000, which is the base address of
    // the SRAM region in many ARM Cortex-M microcontrollers. This comparison
    // ensures that the stack pointer is within the valid SRAM range.
    fn validate_stack_pointer(addr_ptr: *const u32) -> bool {
        let sp = unsafe { addr_ptr.read() };
        sp & 0xfffe_0000 == 0x2000_0000
    }

    /// Set magic value in RAM so that
    /// DFU would be triggered on next reset.
    fn set_key_to_dfu_boot() {
        unsafe { Self::magic_mut_ptr().write_volatile(Self::KEY_STAY_IN_BOOT) };
    }

    /// Erase magic value in RAM so that
    /// DFU would be triggered only once.
    fn clear_dfu_key() {
        unsafe { Self::magic_mut_ptr().write_volatile(0) };
    }

    /// check if magic value is not set, then device must Jump to APP mode.
    /// NOTE: KEY in memory is cleared after reading
    pub fn should_enter_app() -> bool {
        let enforced_dfu = Self::get_magic_val() == Self::KEY_STAY_IN_BOOT;
        if enforced_dfu {
            Self::clear_dfu_key();
        }
        #[cfg(feature = "defmt")]
        defmt::info!("DFU boot key in memory: {}", enforced_dfu);
        // negate the value because we want to enter DFU mode if the key is set
        !enforced_dfu
    }

    #[inline(never)]
    pub fn try_jump_to_app() {
        let vt = FIRMWARE_ADDRESS as *const u32;
        if Self::validate_stack_pointer(vt) {
            #[cfg(feature = "defmt")]
            defmt::info!("Trying to start app");
            // let vt = FW_ADDRESS as *const u32;

            cortex_m::interrupt::disable();

            unsafe {
                let peripherals = cortex_m::peripheral::Peripherals::steal();
                peripherals.SCB.vtor.write(vt as u32);
                cortex_m::asm::bootload(vt);
            }
        };
    }

    /// Disables all interrupts and resets the device to bootloader mode with
    /// key set in memory. Can be used if you want to enter DFU mode from
    /// the application
    pub fn reset_to_bootloader() -> ! {
        cortex_m::interrupt::disable();
        Self::set_key_to_dfu_boot();
        Self::controller_reset();
    }

    pub fn controller_reset() -> ! {
        cortex_m::peripheral::SCB::sys_reset();
    }
}
