#![no_std]
#![no_main]

use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};
use esp32c3_hal as hal;
use esp32c3_hal::{delay::McycleDelay, gpio::GpioExt};
use hal::pac;
use panic_halt as _;
// use longan_nano::led::{rgb, Led};
use riscv_rt::entry;
// use smart_leds::SmartLedsWrite;
// use ws2812_timer_delay::Ws2812;

#[entry]
fn main() -> ! {
	let dp = pac::Peripherals::take().unwrap();

	let gpio = dp.GPIO.split();

	let mut pin = gpio.p8.into_push_pull_output();

	let mut delay = McycleDelay::new();

	// let ws2812 = Ws2812::new(delay, pin);

	// let mut i = 0;
	loop {
		// let inext = (i + 1) % leds.len();
		// leds[i].off();
		// leds[inext].on();
		pin.set_high().unwrap();
		delay.delay_ms(500);
		pin.set_low().unwrap();
		delay.delay_ms(500);

		// i = inext;
	}
}
