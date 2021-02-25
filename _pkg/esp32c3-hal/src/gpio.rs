//! General Purpose Input / Output

use core::marker::PhantomData;

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    fn split(self) -> Self::Parts;
}

/// Unknown mode (type state)
pub struct Unknown {}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Input mode (type state)
pub struct RTCInput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating Input (type state)
pub struct Floating;

/// Pulled down Input (type state)
pub struct PullDown;

/// Pulled up Input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Output mode via RTC (type state)
pub struct RTCOutput<MODE> {
    _mode: PhantomData<MODE>,
}

/// Totem Pole aka Push-Pull output (type state)
pub struct PushPull;

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

/// Alternate function
pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}

/// Alternate Function 1
pub struct AF1;

/// Alternate Function 2
pub struct AF2;

/// Alternate Function 2
pub struct AF3;

/// Drive strength (values are approximates)
pub enum DriveStrength {
    I5mA = 0,
    I10mA = 1,
    I20mA = 2,
    I40mA = 3,
}

/// Alternative pin functions
#[derive(PartialEq)]
pub enum AlternateFunction {
    Function1 = 0,
    Function2 = 1,
    Function3 = 2,
    Function4 = 3,
    Function5 = 4,
    Function6 = 5,
}

pub enum State {
    High,
    Low,
}

// trait PeripheralAccess {
//     fn peripheral() -> &'static crate::pac::gpio::RegisterBlock;
//
//     // fn set_mode(index: u8, mode: PortMode) {
//     // 	assert!(index < 16);
//     //
//     // 	let bits = mode.into_bits();
//     // 	let offset = (index * 4) % 32;
//     // 	let mask = !(0b1111u32 << offset);
//     // 	let value = (bits as u32) << offset;
//     // 	let regs = Self::peripheral();
//     //
//     // 	interrupt::free(|_| {
//     // 		if index < 8 {
//     // 			regs.ctl0.modify(|r, w| unsafe {
//     // 				w.bits((r.bits() & mask) | value)
//     // 			});
//     // 		} else {
//     // 			regs.ctl1.modify(|r, w| unsafe {
//     // 				w.bits((r.bits() & mask) | value)
//     // 			});
//     // 		}
//     // 	});
//     // }
//
//     #[inline(always)]
//     fn set_bit(index: u8) {
//         assert!(index < 16);
//
//         let regs = Self::peripheral();
//
//         // NOTE(unsafe) atomic write to a stateless register
//         // Port bit operate register
//         regs.bop.write(|w| unsafe { w.bits(1u32 << index) });
//     }
//
//     #[inline(always)]
//     fn clear_bit(index: u8) {
//         assert!(index < 16);
//
//         let regs = Self::peripheral();
//
//         // NOTE(unsafe) atomic write to a stateless register
//         regs.bop.write(|w| unsafe { w.bits(1u32 << (16 + index)) });
//     }
//
//     #[inline(always)]
//     fn is_high(index: u8) -> bool {
//         assert!(index < 16);
//
//         let regs = Self::peripheral();
//
//         let mask = 1u32 << index;
//         regs.gpio_status.read().bits() & mask != 0
//     }
//
//     #[inline(always)]
//     fn is_set_high(index: u8) -> bool {
//         assert!(index < 16);
//
//         let regs = Self::peripheral();
//
//         let mask = 1u32 << index;
//         regs.gpio_out.read().bits() & mask != 0
//     }
// }

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
pub enum OutputSignal {
    GPIO = 256,
}

// macro_rules! gpio {
//     ($GPIOX:ident, [
//         $($PXi:ident: ($pxi:ident, $i:expr),)+
//     ]) => {
use crate::pac::GPIO;
use crate::pac::IO_MUX;
use core::convert::Infallible;
use embedded_hal::digital::v2::OutputPin;

/// GPIO parts
pub struct Parts {
    pub p8: P8<Unknown>,
    // $(
    //     /// Pin
    //     pub $pxi: $PXi<Unknown>,
    // )+
}

impl GpioExt for GPIO {
    type Parts = Parts;

    fn split(self) -> Parts {
        // $GPIOX::enable(rcu);
        // $GPIOX::reset(rcu);

        Parts {
            p8: P8 { _mode: PhantomData },
            // $(
            //     $pxi: $PXi { _mode: PhantomData },
            // )+
        }
    }
}

// $(

// pub struct $PXi<MODE> {
pub struct P8<MODE> {
    _mode: PhantomData<MODE>,
}

impl<MODE> OutputPin for P8<Output<MODE>> {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { (*GPIO::ptr()).out_w1tc.write(|w| w.bits(1 << 8)) };
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { (*GPIO::ptr()).out_w1ts.write(|w| w.bits(1 << 8)) };
        Ok(())
    }
}

impl<MODE> P8<MODE> {
    fn init_output(&self, alternate: AlternateFunction, open_drain: bool) {
        let gpio = unsafe { &*GPIO::ptr() };
        let iomux = unsafe { &*IO_MUX::ptr() };

        // out en set: w1ts
        // out en clear: w1tc

        gpio.enable_w1ts.write(|w| unsafe { w.bits(1 << 8) });
        let pin: &crate::pac::gpio::PIN = &gpio.pin[8];
        pin.modify(|_, w| w.pad_driver().bit(open_drain));
        let func: &crate::pac::gpio::FUNC_OUT_SEL_CFG = &gpio.func_out_sel_cfg[8];
        func.modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as u8) });
        // gpio.func_out_sel_cfg[8].modify(|_, w| unsafe { w.out_sel().bits(OutputSignal::GPIO as u16) });

        iomux.gpio8.modify(|_, w| unsafe {
            w.mcu_sel()
                .bits(alternate as u8)
                .fun_ie()
                .clear_bit()
                .fun_pd()
                .clear_bit()
                .fun_pu()
                .clear_bit()
                .fun_drv()
                .bits(DriveStrength::I20mA as u8)
                .slp_sel()
                .clear_bit()
        });
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// Initial state will be low.
    pub fn into_push_pull_output(self) -> P8<Output<PushPull>> {
        self.init_output(AlternateFunction::Function2, false);
        P8 { _mode: PhantomData }
    }
}

// )+

// }
// }

// gpio!([
// 	P0:  (p0,  0),
// 	P1:  (p1,  1),
// 	P2:  (p2,  2),
// 	P3:  (p3,  3),
// 	P4:  (p4,  4),
// 	P5:  (p5,  5),
// 	P6:  (p6,  6),
// 	P7:  (p7,  7),
// 	P8:  (p8,  8),
// 	P9:  (p9,  9),
// 	P10: (p10, 10),
// 	P11: (p11, 11),
// 	P12: (p12, 12),
// 	P13: (p13, 13),
// 	P14: (p14, 14),
// 	P15: (p15, 15),
// 	P16: (p16, 16),
// 	P17: (p17, 17),
// 	P18: (p18, 18),
// 	P19: (p19, 19),
// 	P20: (p20, 20),
// 	P21: (p21, 21),
// 	P22: (p22, 22),
// 	P23: (p23, 23),
// 	P24: (p24, 24),
// 	P25: (p25, 25),
// ]);

// gpio!([
//     PA0: (pa0, 0, Input<Floating>),
//     PA1: (pa1, 1, Input<Floating>),
//     PA2: (pa2, 2, Input<Floating>),
//     PA3: (pa3, 3, Input<Floating>),
//     PA4: (pa4, 4, Input<Floating>),
//     PA5: (pa5, 5, Input<Floating>),
//     PA6: (pa6, 6, Input<Floating>),
//     PA7: (pa7, 7, Input<Floating>),
//     PA8: (pa8, 8, Input<Floating>),
//     PA9: (pa9, 9, Input<Floating>),
//     PA10: (pa10, 10, Input<Floating>),
//     PA11: (pa11, 11, Input<Floating>),
//     PA12: (pa12, 12, Input<Floating>),
//     PA13: (pa13, 13, Debugger),
//     PA14: (pa14, 14, Debugger),
//     PA15: (pa15, 15, Debugger),
// ]);

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpioy:ident, $PXx:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {
            use core::convert::Infallible;
            use core::marker::PhantomData;
            use crate::hal::digital::v2::{OutputPin, InputPin, StatefulOutputPin, toggleable};
            use crate::pac::$GPIOX;
            use crate::rcu::{Rcu, Enable, Reset};
            use super::{
                PeripheralAccess,
                PortMode,
                InputPortConfiguration,
                OutputPortConfiguration,
                Alternate, Floating, GpioExt, Input,
                OpenDrain,
                Output,
                PullDown,
                PullUp,
                PushPull,
                Analog,
                State,
                Active,
                Debugger,
                Pxx
            };

            /// GPIO parts
            pub struct Parts {
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl PeripheralAccess for $GPIOX {
                #[inline(always)]
                fn peripheral() -> &'static crate::pac::gpioa::RegisterBlock {
                    unsafe { &*$GPIOX::ptr() }
                }
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self, rcu: &mut Rcu) -> Parts {
                    $GPIOX::enable(rcu);
                    $GPIOX::reset(rcu);

                    Parts {
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Partially erased pin. Only used in the Pxx enum
            pub struct Generic<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> Generic<MODE> {
                pub fn downgrade(self) -> Pxx<MODE> {
                    Pxx::$PXx(self)
                }
            }

            impl<MODE> OutputPin for Generic<Output<MODE>> {
                type Error = Infallible;
                fn set_high(&mut self) -> Result<(), Self::Error> {
                    $GPIOX::set_bit(self.i);
                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    $GPIOX::clear_bit(self.i);
                    Ok(())
                }
            }

            impl<MODE> InputPin for Generic<Input<MODE>> {
                type Error = Infallible;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    Ok($GPIOX::is_high(self.i))
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    Ok(!$GPIOX::is_high(self.i))
                }
            }


            impl<MODE> StatefulOutputPin for Generic<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    Ok($GPIOX::is_set_high(self.i))
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    Ok(!$GPIOX::is_set_high(self.i))
                }
            }

            impl<MODE> toggleable::Default for Generic<Output<MODE>> {}

            impl InputPin for Generic<Output<OpenDrain>> {
                type Error = Infallible;
                fn is_high(&self) -> Result<bool, Self::Error> {
                    Ok($GPIOX::is_high(self.i))
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    Ok(!$GPIOX::is_high(self.i))
                }
            }

            pub type $PXx<MODE> = Pxx<MODE>;

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl $PXi<Debugger> {
                    /// Put the pin in an active state. The caller
                    /// must enforce that the pin is really in this
                    /// state in the hardware.
                    #[allow(dead_code)]
                    pub(crate) unsafe fn activate(self) -> $PXi<Input<Floating>> {
                        // JTAG/Serial-Wired Debug pins are in input PU/PD mode after reset.
                        // Explicitly convert into floating inputs.
                        let mode = PortMode::Input(InputPortConfiguration::Floating);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Configures the pin to operate as an alternate function push-pull output
                    /// pin.
                    pub fn into_alternate_push_pull(self) -> $PXi<Alternate<PushPull>> {
                        let mode = PortMode::Output50Mhz(OutputPortConfiguration::AfioPushPull);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an alternate function open-drain output
                    /// pin.
                    pub fn into_alternate_open_drain(self) -> $PXi<Alternate<OpenDrain>> {
                        let mode = PortMode::Output50Mhz(OutputPortConfiguration::AfioOpenDrain);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating input pin
                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        let mode = PortMode::Input(InputPortConfiguration::Floating);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down input pin
                    pub fn into_pull_down_input(self) -> $PXi<Input<PullDown>> {
                        $GPIOX::clear_bit($i); // pull down

                        let mode = PortMode::Input(InputPortConfiguration::Pulled);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up input pin
                    pub fn into_pull_up_input(self) -> $PXi<Input<PullUp>> {
                        $GPIOX::set_bit($i); // pull up

                        let mode = PortMode::Input(InputPortConfiguration::Pulled);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// Initial state will be low.
                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        self.into_open_drain_output_with_state(State::Low)
                    }

                    /// Configures the pin to operate as an open-drain output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    pub fn into_open_drain_output_with_state(
                        self,
                        initial_state: State,
                    ) -> $PXi<Output<OpenDrain>> {
                        match initial_state {
                            State::High => $GPIOX::set_bit($i),
                            State::Low  => $GPIOX::clear_bit($i),
                        }

                        let mode = PortMode::Output50Mhz(OutputPortConfiguration::GpioOpenDrain);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// Initial state will be low.
                    pub fn into_push_pull_output(
                        self
                    ) -> $PXi<Output<PushPull>> {
                        self.into_push_pull_output_with_state(State::Low)
                    }

                    /// Configures the pin to operate as an push-pull output pin.
                    /// `initial_state` specifies whether the pin should be initially high or low.
                    pub fn into_push_pull_output_with_state(
                        self,
                        initial_state: State,
                    ) -> $PXi<Output<PushPull>> {
                        match initial_state {
                            State::High => $GPIOX::set_bit($i),
                            State::Low  => $GPIOX::clear_bit($i),
                        }

                        let mode = PortMode::Output50Mhz(OutputPortConfiguration::GpioPushPull);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }


                    /// Configures the pin to operate as an analog input pin
                    pub fn into_analog(self) -> $PXi<Analog> {
                        let mode = PortMode::Input(InputPortConfiguration::Analog);
                        $GPIOX::set_mode($i, mode);

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<MODE> where MODE: Active {
                    /// Erases the pin number from the type
                    fn into_generic(self) -> Generic<MODE> {
                        Generic {
                            i: $i,
                            _mode: self._mode,
                        }
                    }

                    /// Erases the pin number and port from the type
                    ///
                    /// This is useful when you want to collect the pins into an array where you
                    /// need all the elements to have the same type
                    pub fn downgrade(self) -> Pxx<MODE> {
                        self.into_generic().downgrade()
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Infallible;
                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        $GPIOX::set_bit($i);
                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        $GPIOX::clear_bit($i);
                        Ok(())
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        Ok($GPIOX::is_set_high($i))
                    }

                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        Ok(!$GPIOX::is_set_high($i))
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Infallible;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        Ok($GPIOX::is_high($i))
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        Ok(!$GPIOX::is_high($i))
                    }
                }

                impl InputPin for $PXi<Output<OpenDrain>> {
                    type Error = Infallible;
                    fn is_high(&self) -> Result<bool, Self::Error> {
                        Ok($GPIOX::is_high($i))
                    }

                    fn is_low(&self) -> Result<bool, Self::Error> {
                        Ok(!$GPIOX::is_high($i))
                    }
                }
            )+
        }
    }
}

macro_rules! impl_pxx {
    ($(($port:ident :: $pin:ident)),*) => {
        use core::convert::Infallible;
        use embedded_hal::digital::v2::{InputPin, StatefulOutputPin, OutputPin};

        pub enum Pxx<MODE> {
            $(
                $pin($port::Generic<MODE>)
            ),*
        }

        impl<MODE> OutputPin for Pxx<Output<MODE>> {
            type Error = Infallible;
            fn set_high(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_high()),*
                }
            }

            fn set_low(&mut self) -> Result<(), Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.set_low()),*
                }
            }
        }

        impl<MODE> StatefulOutputPin for Pxx<Output<MODE>> {
            fn is_set_high(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_high()),*
                }
            }

            fn is_set_low(&self) -> Result<bool, Self::Error> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> InputPin for Pxx<Input<MODE>> {
            type Error = Infallible;
            fn is_high(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_high()),*
                }
            }

            fn is_low(&self) -> Result<bool, Infallible> {
                match self {
                    $(Pxx::$pin(pin) => pin.is_low()),*
                }
            }
        }
    }
}

// impl_pxx! {
//     (gpioa::PAx),
//     (gpiob::PBx),
//     (gpioc::PCx),
//     (gpiod::PDx),
//     (gpioe::PEx)
// }
//
// gpio!(GPIO, gpio, gpioa, PAx, [
//     PA0: (pa0, 0, Input<Floating>),
//     PA1: (pa1, 1, Input<Floating>),
//     PA2: (pa2, 2, Input<Floating>),
//     PA3: (pa3, 3, Input<Floating>),
//     PA4: (pa4, 4, Input<Floating>),
//     PA5: (pa5, 5, Input<Floating>),
//     PA6: (pa6, 6, Input<Floating>),
//     PA7: (pa7, 7, Input<Floating>),
//     PA8: (pa8, 8, Input<Floating>),
//     PA9: (pa9, 9, Input<Floating>),
//     PA10: (pa10, 10, Input<Floating>),
//     PA11: (pa11, 11, Input<Floating>),
//     PA12: (pa12, 12, Input<Floating>),
//     PA13: (pa13, 13, Debugger),
//     PA14: (pa14, 14, Debugger),
//     PA15: (pa15, 15, Debugger),
// ]);
