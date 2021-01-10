#![feature(
    lang_items,
    panic_info_message,
    fmt_as_str,
    num_as_ne_bytes,
    unsigned_abs
)]
#![no_std]
#![no_main]
#![allow(dead_code, unused_imports)]

use core::prelude::*;
use embedded_hal::prelude::*;
use stm32f1xx_hal::prelude::*;

use embedded_hal::digital::v2::{InputPin, OutputPin};

use cortex_m::peripheral::DWT;

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

use stm32f1xx_hal;
use stm32f1xx_hal::gpio::ExtiPin;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::pac::{Interrupt, CAN1};
use stm32f1xx_hal::{adc, dma, gpio, pwm};

use bxcan::{filter::Mask32, Can, Frame, Id, Rx, StandardId, Tx};

use heapless;
use heapless::consts;
use heapless::pool::singleton::Box;
use heapless::pool::singleton::Pool;
use heapless::BinaryHeap;
use heapless::Vec;

use core::cmp::Ordering;
use core::convert::Into;

use defmt;

/// this makes sure that the rtt logger is linked into the binary
use defmt_rtt as _;

mod can;
mod error_codes;
mod status;

use error_codes::ErrorCode;

use can::PriorityFrame;

#[derive(Eq, PartialEq, Copy, Clone)]
pub enum IdleMode {
    Brake,
    Coast,
}

const HSE_CLOCK_MHZ: u32 = 8;
const SYS_CLOCK_MHZ: u32 = 72;

const HSE_CLOCK_HZ: u32 = HSE_CLOCK_MHZ * 1_000_000;
const SYS_CLOCK_HZ: u32 = SYS_CLOCK_MHZ * 1_000_000;

const CAN_CONFIG: u32 = 0x001e0001;
const CAN_TIMEOUT: u32 = SYS_CLOCK_MHZ * 2;

const V_OFF: f32 = 0.050; // volts
const R_SENSE_VAL: f32 = 0.004; // ohms
const AMP_GAIN: f32 = 20.0; // 20 V / V
const CURRENT_EXTERNAL_SCALE: f32 = 10_000.0 / 22_000.0; // from the current divider

// hardware type defs
type Adc = adc::Adc<pac::ADC1>;
type DmaPayload = adc::AdcPayload<gpio::gpioa::PA3<gpio::Analog>, adc::Continuous>;
type AdcDma = stm32f1xx_hal::dma::RxDma<DmaPayload, dma::dma1::C1>;

type PwmChannel<C> = pwm::PwmChannel<pac::TIM1, C>;
type MotorHighChannel = PwmChannel<pwm::C2>;
type MotorLowChannel = PwmChannel<pwm::C3>;
type CurrentLimitChannel = PwmChannel<pwm::C4>;

type FaultPin = gpio::gpioa::PA4<gpio::Input<gpio::Floating>>;
type OverCurrentPin = gpio::gpioa::PA5<gpio::Input<gpio::Floating>>;
type SleepPin = gpio::gpioa::PA6<gpio::Output<gpio::PushPull>>;

type Status1 = status::StatusLed<gpio::gpiob::PB10<gpio::Output<gpio::PushPull>>>;
type Status2 = status::StatusLed<gpio::gpiob::PB11<gpio::Output<gpio::PushPull>>>;

// timings
const fn times_per_second(times: u32) -> u32 {
    SYS_CLOCK_HZ / times
}

const MOTOR_UPDATE_PD: u32 = times_per_second(1000);
const CAN_VALUE_UPDATE_PD: u32 = times_per_second(20);
const LED_UPDATE_PD: u32 = times_per_second(8);

#[defmt::timestamp]
fn timestamp() -> u64 {
    DWT::get_cycle_count() as u64
}

// can frame pool definition
heapless::pool! {
    #[allow(non_upper_case_globals)]
    CanFramePool: PriorityFrame
}

// can frame pool helper method
fn allocate_tx_frame(frame: Frame) -> Box<CanFramePool, heapless::pool::Init> {
    let b = CanFramePool::alloc().unwrap();
    b.init(PriorityFrame(frame))
}

#[app(device=stm32f1::stm32f103, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        /// all the CAN bus resources
        #[init(None)]
        last_can_rx: Option<Instant>,
        can_id: bxcan::Id,
        can_tx: Tx<stm32f1xx_hal::can::Can<pac::CAN1>>,
        can_rx: Rx<stm32f1xx_hal::can::Can<pac::CAN1>>,
        can_tx_queue: heapless::BinaryHeap<
            Box<CanFramePool, heapless::pool::Init>,
            consts::U16,
            heapless::binary_heap::Max,
        >,

        /// adc values from current sense
        adc_buf: dma::CircBuffer<u16, AdcDma>,
        // adc: adc::Adc<pac::ADC1>,
        /// pwm channels
        motor_high: MotorHighChannel,
        motor_low: MotorLowChannel,
        motor_current_limit: CurrentLimitChannel,

        // gpio
        fault_pin: FaultPin,
        sleep_pin: SleepPin,
        over_current_pin: OverCurrentPin,

        // status
        status1: Status1,
        status2: Status2,

        //software variables
        /// Im not sure if we need this, since the estop cuts the power
        #[cfg(feature = "heartbeat")]
        #[init(None)]
        last_heartbeat: Option<Instant>,

        /// setpoint duty cycle
        /// I made this an i16 instead of a float
        /// because I want to avoid floating point operations
        #[init(0)]
        setpoint: i16,

        #[init(false)]
        inverted: bool,

        /// current in amps
        #[init(10)]
        current_limit: u8,

        #[init(IdleMode::Coast)]
        idle_mode: IdleMode,
    }

    #[init(schedule=[update_leds, motor_update])]
    fn init(cx: init::Context) -> init::LateResources {
        let can_tx_queue = BinaryHeap::new();

        CanFramePool::grow(cortex_m::singleton!(: [u8; 1024] = [0; 1024]).unwrap());

        let mut peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut debug = device.DBGMCU;
        let exti = device.EXTI;

        let clocks = rcc
            .cfgr
            .use_hse(HSE_CLOCK_MHZ.mhz())
            .sysclk(SYS_CLOCK_MHZ.mhz())
            .hclk(SYS_CLOCK_MHZ.mhz())
            .pclk1((SYS_CLOCK_MHZ / 2).mhz())
            .pclk2(SYS_CLOCK_MHZ.mhz())
            .freeze(&mut flash.acr);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // declare this here, put motor driver in sleep mode while we init
        let mut sleep_pin = gpioa
            .pa6
            .into_push_pull_output_with_state(&mut gpioa.crl, gpio::State::Low);

        #[allow(unused_must_use)]
        let can_id = {
            let mut id = 0_u16;
            let mut pins =
                heapless::Vec::<gpio::Pxx<gpio::Input<gpio::PullDown>>, consts::U8>::new();

            // disable jtag
            let (_, pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

            // put all can_id pins in a vec
            pins.push(gpiob.pb0.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb1.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb2.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb7.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb6.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb5.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(pb4.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(pb3.into_pull_down_input(&mut gpiob.crl).downgrade());

            // interate over the pins and shift values as we go
            for (shift, pin) in pins.iter().enumerate() {
                id |= (pin.is_high().unwrap() as u16) << (shift as u16);
                defmt::debug!("Id so far: {:u16}", id);
            }
            pins.clear(); // just to make sure this happens.
            id
        };

        let can_id = StandardId::new(can_id).unwrap();

        let can = stm32f1xx_hal::can::Can::new(device.CAN1, &mut rcc.apb1, device.USB);

        {
            let tx_pin = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
            let rx_pin = gpiob.pb8.into_floating_input(&mut gpiob.crh);

            can.assign_pins((tx_pin, rx_pin), &mut afio.mapr);
        }

        let mut can = bxcan::Can::new(can);

        can.configure(|config| {
            config.set_bit_timing(CAN_CONFIG);

            config.set_silent(false);
            config.set_loopback(false);
        });

        let can_id_mask = StandardId::new(0xFF).unwrap();
        let mut can_filters = can.modify_filters();
        can_filters.enable_bank(0, Mask32::frames_with_std_id(can_id, can_id_mask));
        drop(can_filters);

        use bxcan::Interrupts;
        can.enable_interrupts(
            Interrupts::FIFO0_MESSAGE_PENDING | Interrupts::FIFO0_MESSAGE_PENDING,
        );

        nb::block!(can.enable()).unwrap();

        let can_id = bxcan::Id::Standard(can_id);

        let (can_tx, can_rx) = can.split();

        let (_, mut motor_high, mut motor_low, mut motor_current_limit) = {
            let tim_pins = (
                gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh),
                gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
                gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh),
                gpioa.pa11.into_alternate_push_pull(&mut gpioa.crh),
            );

            let mut tim = stm32f1xx_hal::timer::Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2);

            // make sure the pwm timers still run during debug, or else the motors will stop
            tim.stop_in_debug(&mut debug, false);

            tim.pwm::<stm32f1xx_hal::timer::Tim1NoRemap, _, _, _>(
                tim_pins,
                &mut afio.mapr,
                100.khz(),
            )
            .split()
        };

        motor_high.enable();
        motor_low.enable();
        motor_current_limit.enable();

        motor_high.set_duty(0);
        motor_low.set_duty(0);
        motor_current_limit.set_duty(0);

        let adc_buf = {
            let dma_ch = device.DMA1.split(&mut rcc.ahb).1;
            let ch0 = gpioa.pa3.into_analog(&mut gpioa.crl);

            // setup adc for fast, continous operation.
            let mut adc = stm32f1xx_hal::adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);
            adc.set_continuous_mode(true);
            adc.set_sample_time(adc::SampleTime::T_28);
            adc.set_align(adc::Align::Right); // TODO: Check if this is correct

            // get singleton buffer and start dma
            let buf = cortex_m::singleton!(: [u16; 2] = [0; 2]).unwrap();
            adc.with_dma(ch0, dma_ch).circ_read(buf)
        };

        let mut fault_pin = gpioa.pa4.into_floating_input(&mut gpioa.crl);
        let mut over_current_pin = gpioa.pa5.into_floating_input(&mut gpioa.crl);

        over_current_pin.make_interrupt_source(&mut afio);
        over_current_pin.trigger_on_edge(&exti, gpio::Edge::FALLING); // this has an external pullup
        over_current_pin.enable_interrupt(&exti);

        fault_pin.make_interrupt_source(&mut afio);
        fault_pin.trigger_on_edge(&exti, gpio::Edge::FALLING); // this has an external pullup
        fault_pin.enable_interrupt(&exti);

        // take motor driver out of sleep mode
        sleep_pin.set_high().unwrap();

        let status1 = status::StatusLed::new(gpiob.pb10.into_push_pull_output(&mut gpiob.crh));
        let status2 = status::StatusLed::new(gpiob.pb11.into_push_pull_output(&mut gpiob.crh));

        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        let now = cx.start;
        cx.schedule
            .update_leds(now + LED_UPDATE_PD.cycles())
            .unwrap();
        cx.schedule
            .motor_update(now + MOTOR_UPDATE_PD.cycles())
            .unwrap();

        init::LateResources {
            can_id,
            can_tx_queue,
            can_tx,
            can_rx,
            adc_buf,
            motor_high,
            motor_low,
            motor_current_limit,
            fault_pin,
            over_current_pin,
            sleep_pin,
            status1,
            status2,
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    #[task(priority = 2, binds = EXTI9_5, resources = [can_id, over_current_pin, fault_pin, can_tx_queue])]
    fn exti9_5(cx: exti9_5::Context) {
        use can::IntoWithId;

        let oc_pin = cx.resources.over_current_pin;
        let f_pin = cx.resources.fault_pin;

        let can_id = cx.resources.can_id;
        let mut can_tx_queue = cx.resources.can_tx_queue;

        if oc_pin.check_interrupt() {
            defmt::debug!("Overcurrent interrupt");

            can_tx_queue.lock(|q| {
                q.push(allocate_tx_frame(
                    can::OutgoingFrame::Overcurrent.into_with_id(can_id.clone()),
                ))
                .unwrap();
            });
            oc_pin.clear_interrupt_pending_bit();
        }

        if f_pin.check_interrupt() {
            defmt::debug!("Motor fault interrupt");

            can_tx_queue.lock(|q| {
                q.push(allocate_tx_frame(
                    can::OutgoingFrame::Error(ErrorCode::MotorDriverFault.into())
                        .into_with_id(can_id.clone()),
                ))
                .unwrap();
            });

            f_pin.clear_interrupt_pending_bit();
        }
    }

    #[task(priority = 1, schedule = [motor_update], resources = [last_heartbeat, motor_low, motor_high, motor_current_limit, setpoint, current_limit, inverted, idle_mode])]
    fn motor_update(cx: motor_update::Context) {
        // set pwm signals for setpoint and current limit
        #[cfg(feature = "heartbeat")]
        let last_heartbeat = cx.resources.last_heartbeat;

        let motor_low = cx.resources.motor_low;
        let motor_high = cx.resources.motor_high;
        let motor_current_limit = cx.resources.motor_current_limit;

        let inverted = *cx.resources.inverted;
        let setpoint = *cx.resources.setpoint * (if inverted { -1 } else { 1 });
        let idle_mode = *cx.resources.idle_mode;
        let current_limit = *cx.resources.current_limit;

        let max_duty = motor_low.get_max_duty();

        #[cfg(feature = "heartbeat")]
        let mut stop = if let Some(last) = last_heartbeat {
            last.elapsed() > 0.cycles()
        } else {
            true
        };

        #[cfg(not(feature = "heartbeat"))]
        let mut stop = false;

        if setpoint == 0 {
            stop = true;
        };

        // make sure this works
        if !stop {
            let internal_set = setpoint.unsigned_abs();
            if setpoint >= 0 {
                motor_low.set_duty(0);
                motor_high.set_duty(internal_set);
            } else {
                motor_low.set_duty(internal_set);
                motor_high.set_duty(0);
            }
        } else {
            let internal_set = if idle_mode == IdleMode::Coast {
                0
            } else {
                max_duty
            };
            motor_low.set_duty(internal_set);
            motor_high.set_duty(internal_set);
        }

        // set current limit
        // I wish we could avoid using floating point here
        // this is from the datasheet
        let vref =
            ((current_limit as f32 * AMP_GAIN * R_SENSE_VAL) + V_OFF) * CURRENT_EXTERNAL_SCALE;

        let pwm_val = motor_current_limit.get_max_duty() as f32 * (vref / 3.3);

        motor_current_limit.set_duty(pwm_val as u16);

        // schedule this task again
        cx.schedule
            .motor_update(Instant::now() + MOTOR_UPDATE_PD.cycles())
            .unwrap();
    }

    #[task(priority = 2, binds = DMA1_CHANNEL1, resources=[adc_buf])]
    fn handle_adc(cx: handle_adc::Context) {
        let adc_buf = cx.resources.adc_buf;
        let mut adc_val = 0_u16;
        adc_buf
            .peek(|buf, _| {
                adc_val = *buf;
            })
            .unwrap();

        // We use floats here because the accuracy matters to an extent
        let volts = (adc_val / 1000) as f32;

        // this comes from the data sheet
        let current: f32 = ((volts - V_OFF) / (R_SENSE_VAL * AMP_GAIN)) / CURRENT_EXTERNAL_SCALE;

        defmt::info!("Motor current: {:f32}", current);

        // TODO: Do something with the adc_val here
        // do calculations
        //
        // we should really do something that takes the vref into account
        // such as: let x = adc_val * 1200 / adc.read_vref();
        // but i dont know how to get the vref value through the dma
        // for now we will just do it the dumb way
    }

    #[task(capacity = 16, resources=[can_tx_queue, last_can_rx, inverted, current_limit, setpoint, last_heartbeat] )]
    fn handle_rx_frame(cx: handle_rx_frame::Context, frame: Frame) {
        use can::IncomingFrame;
        use can::IncomingFrame::*;
        use core::convert::TryFrom;

        let mut last_rx = cx.resources.last_can_rx;

        match IncomingFrame::try_from(frame) {
            Ok(Setpoint(setpoint)) => {
                defmt::info!("Setting setpoint to {:i16}", setpoint);
                *cx.resources.setpoint = setpoint;
            }
            Ok(SetCurrentLimit(limit)) => {
                defmt::info!("Setting current limit to {:u8} amps", limit);
                *cx.resources.current_limit = limit;
            }
            Ok(Invert(inv)) => {
                defmt::info!("Setting motor inversion to {:bool}", inv);
                *cx.resources.inverted = inv;
            }
            #[cfg(feature = "heartbeat")]
            Ok(HeartBeat) => {
                defmt::info!("Heartbeat...");
                *cx.resources.last_heartbeat = Some(Instant::now());
            }
            #[cfg(not(feature = "heartbeat"))]
            Ok(HeartBeat) => {}
            Ok(Stop) => {
                defmt::info!("Stopping motor (setpoint = 0)");
                *cx.resources.setpoint = 0;
            }
            Err(e) => defmt::panic!("{:?}", e),
        };

        last_rx.lock(|instant: &mut Option<Instant>| *instant = Some(Instant::now()));
        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    #[task(priority=3, binds = USB_LP_CAN_RX0, resources=[can_rx], spawn=[handle_rx_frame])]
    fn can_rx0(cx: can_rx0::Context) {
        let rx = cx.resources.can_rx;

        loop {
            match rx.receive() {
                Ok(frame) => {
                    cx.spawn.handle_rx_frame(frame).unwrap();
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(_)) => {}
            }
        }
    }

    #[task(priority=3, binds = CAN_RX1)]
    fn can_rx1(_: can_rx1::Context) {
        rtic::pend(Interrupt::USB_LP_CAN_RX0);
    }

    #[task(priority = 3, binds = USB_HP_CAN_TX, resources = [can_tx, can_tx_queue, last_can_rx])]
    fn can_tx(cx: can_tx::Context) {
        let tx = cx.resources.can_tx;
        let tx_queue = cx.resources.can_tx_queue;
        let last_rx = cx.resources.last_can_rx;

        let can_ok = if let Some(t) = last_rx {
            !(t.elapsed() > CAN_TIMEOUT.cycles())
        } else {
            false
        };

        tx.clear_interrupt_flags();

        if !can_ok {
            defmt::debug!("Canbus timeout, waiting for recieved frame before tx");
            return;
        }

        while let Some(frame) = tx_queue.peek() {
            match tx.transmit(&frame.0) {
                Ok(None) => {
                    use core::ops::Deref;
                    let sent_frame = tx_queue.pop();
                    defmt::info!("Sent Frame: {:?}", sent_frame.unwrap().deref().0);
                }
                Ok(Some(pending_frame)) => {
                    tx_queue.pop();
                    tx_queue.push(allocate_tx_frame(pending_frame)).unwrap();
                }
                Err(nb::Error::WouldBlock) => break,
                Err(_) => unreachable!(),
            }
        }
    }

    #[task(priority = 5, resources = [status1, status2], schedule=[update_leds])]
    fn update_leds(cx: update_leds::Context) {
        let status1 = cx.resources.status1;
        let status2 = cx.resources.status2;

        status1.update();
        status2.update();

        cx.schedule
            .update_leds(Instant::now() + LED_UPDATE_PD.cycles())
            .unwrap();
    }

    #[allow(non_snake_case)]
    extern "C" {
        fn USART2();

        fn SPI2();
        fn SPI3();
    }
};

#[panic_handler]
#[inline(never)]
fn my_panic(info: &core::panic::PanicInfo) -> ! {
    use core::fmt::Debug;

    use defmt::{consts, Debug2Format};

    defmt::error!(
        "Panic: \"{:?}\" \nin file {:str} at line {:u32}",
        Debug2Format::<consts::U1024>(info.message().unwrap()),
        info.location().unwrap().file(),
        info.location().unwrap().line()
    );
    loop {}
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {}
}

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}
