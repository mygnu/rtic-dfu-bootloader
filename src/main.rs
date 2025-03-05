#![no_main]
#![no_std]

#[rtic::app(device = stm32f1xx_hal::pac)]
mod app {
    use core::{ptr::addr_of_mut, str};
    #[cfg(not(feature = "defmt"))]
    use panic_halt as _;
    #[cfg(feature = "defmt")]
    use rdb_lib::feature_defmt as _;
    use rdb_lib::{
        BOOTLOADER_SIZE_BYTES, FLASH_SIZE, FLASH_SIZE_BYTES, FW_ADDRESS,
        KEY_STAY_IN_BOOT,
    };
    use rtic_monotonics::systick::prelude::*;
    use stm32f1xx_hal::flash;
    use stm32f1xx_hal::gpio::{Output, PC13, PinState, PushPull};
    use stm32f1xx_hal::pac::GPIOB;
    use stm32f1xx_hal::pac::RCC;
    use stm32f1xx_hal::prelude::*;
    use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
    use usb_device::device::{
        StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid,
    };
    use usbd_dfu::{DFUClass, DFUManifestationError, DFUMemError, DFUMemIO};

    fn controller_reset() -> ! {
        cortex_m::peripheral::SCB::sys_reset();
    }

    pub struct STM32Mem {
        flash: flash::Parts,
        buffer: [u8; 128],
    }

    impl STM32Mem {
        pub fn new(flash: flash::Parts) -> Self {
            STM32Mem {
                flash,
                buffer: [0; 128],
            }
        }

        pub fn writer(&mut self) -> flash::FlashWriter {
            self.flash.writer(flash::SectorSize::Sz1K, FLASH_SIZE)
        }
    }

    impl DFUMemIO for STM32Mem {
        const INITIAL_ADDRESS_POINTER: u32 = flash::FLASH_START;
        const PROGRAM_TIME_MS: u32 = 7; // time it takes to program 128 bytes
        const ERASE_TIME_MS: u32 = 50;
        const FULL_ERASE_TIME_MS: u32 = 50 * 64;

        const MEM_INFO_STRING: &'static str = "@Flash/0x08000000/16*1Ka,48*1Kg";
        const HAS_DOWNLOAD: bool = true;
        const HAS_UPLOAD: bool = true;

        fn read(
            &mut self,
            address: u32,
            length: usize,
        ) -> core::result::Result<&[u8], DFUMemError> {
            let flash_top: u32 =
                Self::INITIAL_ADDRESS_POINTER + FLASH_SIZE_BYTES as u32;

            if address < Self::INITIAL_ADDRESS_POINTER {
                return Err(DFUMemError::Address);
            }
            if address >= flash_top {
                return Ok(&[]);
            }

            let len = length.min((flash_top - address) as usize);

            let mem = unsafe {
                &*core::ptr::slice_from_raw_parts(address as *const u8, len)
            };

            Ok(mem)
        }

        fn erase(
            &mut self,
            address: u32,
        ) -> core::result::Result<(), DFUMemError> {
            if address < flash::FLASH_START {
                return Err(DFUMemError::Address);
            }

            if address < flash::FLASH_START + BOOTLOADER_SIZE_BYTES {
                return Err(DFUMemError::Address);
            }

            if address >= flash::FLASH_START + FLASH_SIZE_BYTES as u32 {
                return Err(DFUMemError::Address);
            }

            if address & (1024 - 1) != 0 {
                return Ok(());
            }

            match self.writer().page_erase(address - flash::FLASH_START) {
                Ok(_) => Ok(()),
                Err(flash::Error::EraseError) => Err(DFUMemError::Erase),
                Err(flash::Error::VerifyError) => Err(DFUMemError::CheckErased),
                Err(_) => Err(DFUMemError::Unknown),
            }
        }

        fn erase_all(&mut self) -> Result<(), DFUMemError> {
            Err(DFUMemError::Unknown)
        }

        fn store_write_buffer(
            &mut self,
            src: &[u8],
        ) -> core::result::Result<(), ()> {
            self.buffer[..src.len()].copy_from_slice(src);
            Ok(())
        }

        fn program(
            &mut self,
            address: u32,
            length: usize,
        ) -> core::result::Result<(), DFUMemError> {
            if address < flash::FLASH_START {
                return Err(DFUMemError::Address);
            }

            let offset = address - flash::FLASH_START;

            if offset < BOOTLOADER_SIZE_BYTES {
                return Err(DFUMemError::Address);
            }

            if offset as usize >= FLASH_SIZE_BYTES - length {
                return Err(DFUMemError::Address);
            }

            // not ideal but we cant have a mutable reference to self.buffer
            let mut data = [0u8; 128];
            data[..length].copy_from_slice(&self.buffer[..length]);

            match self.writer().write(offset, &data) {
                Ok(_) => Ok(()),
                Err(flash::Error::ProgrammingError) => Err(DFUMemError::Prog),
                Err(flash::Error::LengthNotMultiple2) => Err(DFUMemError::Prog),
                Err(flash::Error::LengthTooLong) => Err(DFUMemError::Prog),
                Err(flash::Error::AddressLargerThanFlash) => {
                    Err(DFUMemError::Address)
                }
                Err(flash::Error::AddressMisaligned) => {
                    Err(DFUMemError::Address)
                }
                Err(flash::Error::WriteError) => Err(DFUMemError::Write),
                Err(flash::Error::VerifyError) => Err(DFUMemError::Verify),
                Err(_) => Err(DFUMemError::Unknown),
            }
        }

        fn manifestation(&mut self) -> Result<(), DFUManifestationError> {
            controller_reset();
        }
    }

    /// Return device serial based on U_ID registers.
    fn read_serial(serial: &mut [u8; 8]) {
        let u_id0 = 0x1FFF_F7E8 as *const u32;
        let u_id1 = 0x1FFF_F7EC as *const u32;

        // Safety: u_id0 and u_id1 are readable stm32f103 registers.
        let sn = unsafe { u_id0.read().wrapping_add(u_id1.read()) };
        fn hex(v: u8) -> u8 {
            match v {
                0..=9 => v + b'0',
                0xa..=0xf => v - 0xa + b'a',
                _ => b' ',
            }
        }

        for (i, d) in serial.iter_mut().enumerate() {
            *d = hex(((sn >> (i * 4)) & 0xf) as u8)
        }
    }

    fn magic_mut_ptr() -> *mut u32 {
        unsafe extern "C" {
            #[link_name = "_dfu_magic"]
            static mut magic: u32;
        }
        addr_of_mut!(magic)
    }

    /// Read magic value to determine if
    /// device must enter DFU mode.
    fn get_uninit_val() -> u32 {
        unsafe { magic_mut_ptr().read_volatile() }
    }

    /// Erase magic value in RAM so that
    /// DFU would be triggered only once.
    fn clear_uninit_val() {
        unsafe { magic_mut_ptr().write_volatile(0) };
    }

    /// Return true if "uninit" area of RAM has a
    /// special value. Used to force DFU mode from
    /// a main firmware programmatically.
    fn dfu_ram_requested() -> bool {
        let stay = get_uninit_val() == KEY_STAY_IN_BOOT;
        if stay {
            clear_uninit_val();
        }
        stay
    }

    /// Check if DFU force external condition.
    /// Check BOOT1 jumper position.
    fn dfu_enforced() -> bool {
        // check BOOT1, PB2 state
        let enforced =
            unsafe { (*GPIOB::ptr()).idr.read().idr2().bit_is_set() };
        #[cfg(feature = "defmt")]
        defmt::info!("Checking BOOT1 {}", enforced);
        enforced
    }

    fn minimal_init() {
        unsafe {
            // enable PWR, AFIO, GPIOB
            (*RCC::ptr()).apb1enr.modify(|_, w| w.pwren().set_bit());
            (*RCC::ptr())
                .apb2enr
                .modify(|_, w| w.afioen().set_bit().iopben().set_bit());
        }

        unsafe {
            // P2 - Input, Floating
            (*GPIOB::ptr())
                .crl
                .modify(|_, w| w.mode2().input().cnf2().open_drain());
        }

        cortex_m::asm::delay(100);
    }

    /// Reset registers that were used for a
    /// check if DFU mode must be enabled to a
    /// default values before starting main firmware.
    fn quick_uninit() {
        unsafe {
            (*GPIOB::ptr()).crl.reset();
            (*RCC::ptr()).apb1enr.reset();
            (*RCC::ptr()).apb2enr.reset();
        }
    }

    /// Initialize stack pointer and jump to a main firmware.
    #[inline(never)]
    fn jump_to_app() -> ! {
        let vt = FW_ADDRESS as *const u32;

        cortex_m::interrupt::disable();

        unsafe {
            let periph = cortex_m::peripheral::Peripherals::steal();
            periph.SCB.vtor.write(vt as u32);
            cortex_m::asm::bootload(vt);
        }
    }

    /// Check if FW looks OK and jump to it, or return.
    fn try_start_app() {
        #[cfg(feature = "defmt")]
        defmt::info!("Trying to start app");
        let sp = unsafe { (FW_ADDRESS as *const u32).read() };
        if sp & 0xfffe_0000 == 0x2000_0000 {
            quick_uninit();
            jump_to_app();
        }
    }

    rtic_monotonics::systick_monotonic!(Mono, 1000);

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        // led: PA8<Output<PushPull>>,
        led: PC13<Output<PushPull>>,
        usb_device: UsbDevice<'static, UsbBusType>,
        usb_dfu: DFUClass<UsbBusType, STM32Mem>,
    }

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None, serial_number: [u8; 8] = [0; 8]])]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        if !dfu_ram_requested() {
            minimal_init();
            if !dfu_enforced() {
                try_start_app();
            }
            quick_uninit();
        }

        let dp = cx.device;
        let mut flash = dp.FLASH.constrain();
        let rcc = dp.RCC.constrain();

        // Setup clocks
        let _clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .hclk(72.MHz())
            .pclk1(36.MHz())
            .pclk2(72.MHz())
            .sysclk(72.MHz())
            .freeze(&mut flash.acr);

        // Initialize the systick interrupt
        Mono::start(cx.core.SYST, 72_000_000); // default STM32F301 clock-rate is 36MHz

        let mut gpioa = dp.GPIOA.split();
        // let led = gpioa.pa8.into_push_pull_output(&mut gpioa.crh);

        let mut gpioc = dp.GPIOC.split();
        let led = gpioc
            .pc13
            .into_push_pull_output_with_state(&mut gpioc.crh, PinState::High);

        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        cortex_m::asm::delay(1024);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb_periph = Peripheral {
            usb: dp.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };
        cx.local.usb_bus.replace(UsbBus::new(usb_periph));

        let stm32mem = STM32Mem::new(flash);

        let usb_dfu =
            DFUClass::new(&cx.local.usb_bus.as_ref().unwrap(), stm32mem);

        read_serial(&mut cx.local.serial_number);

        let usb_device = UsbDeviceBuilder::new(
            cx.local.usb_bus.as_ref().unwrap_or_else(|| panic!()),
            UsbVidPid(0x1209, 0x2444),
        )
        .strings(&[StringDescriptors::default()
            .manufacturer("Hematite Engineering")
            .product("USB DFU Bootloader")
            .serial_number(unsafe {
                str::from_utf8_unchecked(cx.local.serial_number)
            })])
        .unwrap_or_else(|_| panic!())
        .device_release(0x0200)
        .self_powered(false)
        .max_power(250)
        .unwrap_or_else(|_| panic!())
        .max_packet_size_0(64)
        .unwrap_or_else(|_| panic!())
        .build();
        #[cfg(feature = "defmt")]
        defmt::info!("USB DFU Bootloader");
        blink::spawn().ok();

        (
            Shared {},
            Local {
                led,
                usb_dfu,
                usb_device,
            },
        )
    }

    #[task(binds = USB_LP_CAN_RX0, local = [usb_device, usb_dfu])]
    fn usb_rx0(cx: usb_rx0::Context) {
        cx.local.usb_device.poll(&mut [cx.local.usb_dfu]);
    }

    #[task(local = [led])]
    async fn blink(cx: blink::Context) {
        loop {
            #[cfg(feature = "defmt")]
            defmt::info!("Blinking");
            cx.local.led.set_low();
            Mono::delay(200.millis()).await;
            cx.local.led.set_high();
            Mono::delay(780.millis()).await;
        }
    }
}
