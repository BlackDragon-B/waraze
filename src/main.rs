#![no_std]
#![no_main]

use panic_halt as _;

use rp_pico::{
    entry,
    hal::{self, pac},
    XOSC_CRYSTAL_FREQ,
};
// USB Device support
use usb_device::{class_prelude::*, prelude::*};

// USB PicoTool Class Device support
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    #[cfg(feature = "rp2040-hal/rp2040-e5")]
    {
        let sio = hal::Sio::new(pac.SIO);
        let _pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB PicoTool Class Device driver

    let mut serial = SerialPort::new(&usb_bus);
    let mut serial2 = SerialPort::new(&usb_bus);

    // Create a USB device RPI Vendor ID and on of these Product ID:
    // https://github.com/raspberrypi/picotool/blob/master/picoboot_connection/picoboot_connection.c#L23-L27
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000A))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Picotool port")
            .serial_number("TEST")])
        .expect("Failed to set strings")
        .composite_with_iads()
        .max_packet_size_0(64).unwrap()
        .build();

        loop {

        if usb_dev.poll(&mut [&mut serial, &mut serial2]) {
            //     let mut buf = [0u8; 64];
            //     match serial.read(&mut buf) {
            //         Err(_e) => {
            //             // Do nothing
            //         }
            //         Ok(0) => {
            //             // Do nothing
            //         }
            //         Ok(count) => {
            //             // Convert to upper case
            //             buf.iter_mut().take(count).for_each(|b| {
            //                 if *b == 0x72 {
            //                     // if req.value & 0x100 != 0 {
            //                     //     gpio_mask = 1 << (req.value >> 9);
            //                     // }
            //                     rp2040_hal::rom_data::reset_to_usb_boot(
            //                         0,
            //                         0,
            //                     );
            //                     // no-need to accept/reject, we'll reset the device anyway
            //                     unreachable!()            
            //                 }
            //                 b.make_ascii_uppercase();
            //             });
            //             // Send back to the host
            //             let mut wr_ptr = &buf[..count];
            //             while !wr_ptr.is_empty() {
            //                 match serial.write(wr_ptr) {
            //                     Ok(len) => wr_ptr = &wr_ptr[len..],
            //                     // On error, just drop unwritten data.
            //                     // One possible error is Err(WouldBlock), meaning the USB
            //                     // write buffer is full.
            //                     Err(_) => break,
            //                 };
            //             }
            //         }
            //     }
            //
        }
    }
}