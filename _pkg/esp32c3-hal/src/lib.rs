#![no_std]

pub use esp32c3_pac as pac;

use embedded_hal as hal;

pub mod gpio;
pub mod time;
pub mod delay;

// #[cfg(feature = "rt")]
// #[doc(hidden)]
// #[no_mangle]
// #[link_section = ".init.rust"]
// #[export_name = "_start_rust_esp32c3"]
// pub unsafe extern "C" fn start_rust_esp32c3() -> ! {
// 	#[rustfmt::skip]
// 	extern "Rust" {
// 		// This symbol will be provided by the user via `#[entry]`
// 		fn main() -> !;
//
// 		// This symbol will be provided by the user via `#[pre_init]`
// 		fn __pre_init();
//
// 		fn _setup_interrupts();
//
// 		fn _mp_hook() -> bool;
// 	}
//
// 	// r0::zero_bss();
//
// 	riscv_rt::start_rust()
// }
