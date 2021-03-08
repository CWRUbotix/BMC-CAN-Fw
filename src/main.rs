#![feature(lang_items, panic_info_message, num_as_ne_bytes)]
#![no_std]
#![no_main]

use embedded_hal::prelude::*;
use stm32f1xx_hal::prelude::*;

use embedded_hal::digital::v2::{InputPin, OutputPin};

use cortex_m::peripheral::DWT;

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

use stm32f1xx_hal::gpio::{ExtiPin, IOPinSpeed, OutputSpeed};
use stm32f1xx_hal::pac;
use stm32f1xx_hal::pac::Interrupt;
use stm32f1xx_hal::{adc, can, dma, gpio, pwm, timer};

use bxcan::{filter::Mask32, Frame, Rx, StandardId, Tx};

use heapless::consts;
use heapless::pool::singleton::Box;
use heapless::pool::singleton::Pool;
use heapless::BinaryHeap;

use core::convert::Into;

/// this makes sure that the rtt logger is linked into the binary
use defmt_rtt as _;

mod can_types;
mod error_codes;
mod idle_mode;
mod status;

use idle_mode::IdleMode;

use error_codes::ErrorCode;

use can_types::PriorityFrame;

/// Clock speed in mhz of the HSE
const HSE_CLOCK_MHZ: u32 = 8;

/// System clock speed after scalars in mhz
const SYS_CLOCK_MHZ: u32 = 72;

/// Clock speed of hse in hz
#[allow(dead_code)]
const HSE_CLOCK_HZ: u32 = HSE_CLOCK_MHZ * 1_000_000;

/// System clock after scalars in hz
const SYS_CLOCK_HZ: u32 = SYS_CLOCK_MHZ * 1_000_000;

/// CAN transmitter configuration byte.
/// This was generated from <http://www.bittiming.can-wiki.info/#bxCAN>
/// We use a 1 Mbs configuration
const CAN_CONFIG: u32 = 0x001e0001;

/// Can timeout constant
/// This represents the number of cycles before we determine that the CAN bus has disconnected
/// For now, 2 seconds seems fine
const CAN_TIMEOUT: u32 = SYS_CLOCK_MHZ * 2;

/// The motor deadband
/// For now, a 1% deadband seems fine
const MOTOR_DEADBAND_PERCENT: f32 = 0.01;

/// Motor deadband multiplied by the max duty cycle.
/// Any duty cycle below this value will be set to 0.
const DEFAULT_MOTOR_DEADBAND: i16 = (i16::MAX as f32 * MOTOR_DEADBAND_PERCENT) as i16;

// The following values are from the motor driver datasheet
/// `V_OFF` is the value output by the current amplifier when no current is detected
/// It's basically an offset. We can theoretically adjust this, but I dont think it matters
const V_OFF: f32 = 0.050; // volts

/// The value of the current sensing shunt resistor in ohms
const R_SENSE_VAL: f32 = 0.004; // ohms

/// The gain of the current amplifier  (in Volts / Volt)
const AMP_GAIN: f32 = 20.0;

/// The scaling factor of the current sense voltage divider.
/// This also represents the scaling factor of our shop current calculations
const CURRENT_EXTERNAL_SCALE: f32 = 22_000.0 / (22_000.0 + 10_000.0); // from the current divider

const CAN_QUEUE_DEPTH: usize = 128;
const CAN_QUEUE_BYTES: usize = core::mem::size_of::<PriorityFrame>() * CAN_QUEUE_DEPTH;

// hardware type defs (these are all self explanatory)
type DmaPayload = adc::AdcPayload<gpio::gpioa::PA3<gpio::Analog>, adc::Continuous>;
type AdcDma = stm32f1xx_hal::dma::RxDma<DmaPayload, dma::dma1::C1>;

const ADC_BUF_LEN: usize = 64;
type AdcBuf = [u16; ADC_BUF_LEN]; // thicc buffer

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
/// This method calculates the number of cycles needed to run a certain periodic
/// function x times per second
const fn times_per_second(times: u32) -> u32 {
    SYS_CLOCK_HZ / times
}

/// Motor update period.
/// 1kHz should be fine since we arent doing anything fancy
const MOTOR_UPDATE_PD: u32 = times_per_second(500);

/// Can update period.
/// We send the current duty cycle and current value here
const CAN_VALUE_UPDATE_PD: u32 = times_per_second(20);

/// How often we update the leds
/// This subsequently controls how fast the leds will flash
const LED_UPDATE_PD: u32 = times_per_second(8);

heapless::pool! {
    /// can frame pool definition.
    /// Allows us to allocate frames similar to allocating on the heap of a normal system
    #[allow(non_upper_case_globals)]
    CanFramePool: PriorityFrame
}

/// Helper function for `CanFramePool`.
/// Allocates a `bxcan::Frame` in our memory pool
fn allocate_tx_frame(frame: Frame) -> Box<CanFramePool, heapless::pool::Init> {
    let b = CanFramePool::alloc().unwrap();
    b.init(PriorityFrame(frame))
}

#[app(device=stm32f1xx_hal::stm32, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        /// The time we last recieved an can frame from the bus
        /// Used to time out our can transiciever
        #[init(None)]
        last_can_rx: Option<Instant>,

        /// The Can id of this board,
        /// calculated using the dip switches on the board
        can_id: bxcan::Id,

        /// Can transmitter
        can_tx: Tx<stm32f1xx_hal::can::Can<pac::CAN1>>,

        /// Can reciever
        can_rx: Rx<stm32f1xx_hal::can::Can<pac::CAN1>>,

        /// Queue for outgoing can frames.
        /// This will send the frames in order of priority,
        /// such as in the CAN standard (lower id means higher priority)
        can_tx_queue: heapless::BinaryHeap<
            Box<CanFramePool, heapless::pool::Init>,
            consts::U16,
            heapless::binary_heap::Max,
        >,

        /// circular adc buffer. This allows us to coninuously read from the
        /// current sensing circuit without interruption. Data will be stored in
        /// two "Halves" of the buffer. One will be filled while the other is being processed.
        adc_buf: dma::CircBuffer<AdcBuf, AdcDma>,

        // adc: adc::Adc<pac::ADC1>,
        // adc_pin: gpio::gpioa::PA3<gpio::Analog>,
        /// Motor high pwm channel. Controls forward duty cycle.
        motor_high: MotorHighChannel,

        /// Motor low pwm channel. Controls reverse duty cycle.
        motor_low: MotorLowChannel,

        /// Pwm channel thats converted to a constant voltage used to set chop current
        motor_current_limit: CurrentLimitChannel,

        /// Gpio pin that signals motor controller faults
        /// Will activate interrupt when signal goes low.
        fault_pin: FaultPin,

        /// Gpio output pin that allows us to set the sleep mode of the
        /// motor controller chip. If we set this output low, the motor controller
        /// chip will be put into sleep mode
        sleep_pin: SleepPin,

        /// Gpio input that signals an overcurrent event.
        /// Will trigger an interrupt when pulled low
        over_current_pin: OverCurrentPin,

        /// First status LED
        status1: Status1,

        /// Second status LED
        status2: Status2,

        /// Last heartbeat recieved. If we dont recieve a heartbeat for a certain time
        /// period, we should stop the motor
        /// NOTE: Im not sure if we need this, since the estop cuts the power
        #[cfg(feature = "heartbeat")]
        #[init(None)]
        last_heartbeat: Option<Instant>,

        /// duty cycle setpoint
        /// I made this an i16 instead of a float
        /// because I want to avoid floating point operations
        #[init(0)]
        setpoint: i16,

        /// Variable indicating motor inversion
        /// if this value is set to true, all setpoints recieved are negated
        #[init(false)]
        inverted: bool,

        /// current limit in amps
        #[init(10)]
        current_limit: u8,

        /// Idle mode variable. This value is used when motor setpoint is 0 or below a certain
        /// threshold
        #[init(IdleMode::Coast)]
        idle_mode: IdleMode,

        /// Most recent current value in amps
        #[init(0.0)]
        current_now: f32,

        /// Most recent duty cycle value
        #[init(0)]
        duty_now: i16,
    }

    /// Initialization function
    /// Here we initialize hardware peripherals and setup software variables
    #[init(schedule=[led_update, motor_update, can_tx])]
    fn init(cx: init::Context) -> init::LateResources {
        // Create tx queue
        let can_tx_queue = BinaryHeap::new();

        // grow our can memory pool
        CanFramePool::grow(
            cortex_m::singleton!(: [u8; CAN_QUEUE_BYTES] = [0; CAN_QUEUE_BYTES]).unwrap(),
        );

        // take peripheral and device instances
        let mut peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        // https://github.com/probe-rs/probe-rs/issues/350#issuecomment-742825415
        device.DBGMCU.cr.modify(|_, w| {
            w.dbg_sleep().set_bit();
            w.dbg_standby().set_bit();
            w.dbg_stop().set_bit()
        });
        device.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());
        device.RCC.ahbenr.modify(|_, w| w.dma2en().enabled());

        // take seperate peripheral instances for future initialization purposes
        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut debug = device.DBGMCU;
        let exti = device.EXTI;

        // configure the system clocks
        let clocks = rcc
            .cfgr
            .use_hse(HSE_CLOCK_MHZ.mhz())
            .sysclk(SYS_CLOCK_MHZ.mhz())
            .hclk(SYS_CLOCK_MHZ.mhz())
            .pclk1((SYS_CLOCK_MHZ / 2).mhz())
            .pclk2(SYS_CLOCK_MHZ.mhz())
            .adcclk(5.mhz())
            .freeze(&mut flash.acr);

        // take gpio instances
        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // initalize motor sleep pin, pull this low before motor does something weird
        let mut sleep_pin = gpioa
            .pa6
            .into_push_pull_output_with_state(&mut gpioa.crl, gpio::State::Low);
        sleep_pin.set_speed(&mut gpioa.crl, gpio::IOPinSpeed::Mhz2); // save some power

        // get the system can id from the dip switches
        #[allow(unused_must_use)]
        let can_id = {
            let mut id = 0_u16;
            let mut pins =
                heapless::Vec::<gpio::Pxx<gpio::Input<gpio::PullDown>>, consts::U8>::new();

            // disable jtag. If we dont do this, we will have issues
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
            }
            pins.clear(); // just to make sure this happens.
            id
        };

        defmt::info!("Can Id: {=u16}", can_id);

        // wrap the can id
        let can_id = StandardId::new(can_id).unwrap();

        // create can peripheral instance
        let can = can::Can::new(device.CAN1, &mut rcc.apb1, device.USB);

        // assign pins to can interface
        {
            let tx_pin = gpiob.pb9.into_alternate_push_pull(&mut gpiob.crh);
            let rx_pin = gpiob.pb8.into_floating_input(&mut gpiob.crh);

            can.assign_pins((tx_pin, rx_pin), &mut afio.mapr);
        }

        // create bxcan can instance
        let mut can = bxcan::Can::new(can);

        // set can config
        can.configure(|config| {
            config.set_bit_timing(CAN_CONFIG);

            // we dont need these at alll. Dont bother
            config.set_silent(false);
            config.set_loopback(false);
        });

        // create mask, we only care about the first byte of the id
        // let can_id_mask = StandardId::new(0xFF).unwrap();

        // configure filters with our settings
        {
            let can_id_mask = bxcan::ExtendedId::new(0xFF).unwrap();
            let ext_id = bxcan::ExtendedId::new(can_id.as_raw().into()).unwrap();
            let mut can_filters = can.modify_filters();
            can_filters.enable_bank(
                0,
                Mask32::frames_with_std_id(can_id, can_id_mask.standard_id()),
            );
            can_filters.enable_bank(1, Mask32::frames_with_ext_id(ext_id, can_id_mask));
        }

        // enable desired interrupts
        use bxcan::Interrupts;
        can.enable_interrupts(
            Interrupts::FIFO0_MESSAGE_PENDING | Interrupts::FIFO1_MESSAGE_PENDING,
        );

        // enable can interface
        // this would block, so we must tell the program this is ok
        nb::block!(can.enable()).unwrap();

        // wrap can id again
        let can_id = bxcan::Id::Standard(can_id);

        // split can peripheral into tx and rx
        let (can_tx, can_rx) = can.split();

        // create the pwm channels
        let (_, mut motor_high, mut motor_low, mut motor_current_limit) = {
            let tim_pins = (
                gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh),
                gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh),
                gpioa.pa10.into_alternate_push_pull(&mut gpioa.crh),
                gpioa.pa11.into_alternate_push_pull(&mut gpioa.crh),
            );

            let mut tim = timer::Timer::tim1(device.TIM1, &clocks, &mut rcc.apb2);

            // make sure the pwm timers still run during debug, or else the motors will stop
            tim.stop_in_debug(&mut debug, false);

            tim.pwm::<stm32f1xx_hal::timer::Tim1NoRemap, _, _, _>(
                tim_pins,
                &mut afio.mapr,
                100.khz(),
            )
            .split()
        };

        // enable pwm channels
        motor_high.enable();
        motor_low.enable();
        motor_current_limit.enable();

        // set duty of all 3 channels to 0 for now
        motor_high.set_duty(0);
        motor_low.set_duty(0);
        motor_current_limit.set_duty(0);

        // setup dma transfer for current sensing circuit
        let adc_buf = {
            // split a dma channel
            let mut dma_ch = device.DMA1.split(&mut rcc.ahb).1;
            dma_ch.listen(dma::Event::HalfTransfer);
            dma_ch.listen(dma::Event::TransferComplete);

            // get our desired analog pin
            let ch0 = gpioa.pa3.into_analog(&mut gpioa.crl);

            // setup adc for fast, continous operation.
            let mut adc = adc::Adc::adc1(device.ADC1, &mut rcc.apb2, clocks);

            // set continous mode, this means the dma will run continuosly
            adc.set_continuous_mode(true);
            adc.set_sample_time(adc::SampleTime::T_239); // NOTE: we can make this faster or slower if we want
            adc.set_align(adc::Align::Right); // TODO: Check if this is correct

            // get singleton buffer and start dma
            let buf = cortex_m::singleton!(: [AdcBuf ; 2] = [[0 ; ADC_BUF_LEN] ; 2]).unwrap();

            // start circular read
            let adc_dma = adc.with_dma(ch0, dma_ch);
            adc_dma.circ_read(buf)
        };

        // create fault and overcurrent pins
        let mut fault_pin = gpioa.pa4.into_floating_input(&mut gpioa.crl);
        let mut over_current_pin = gpioa.pa5.into_floating_input(&mut gpioa.crl);

        // setup over current interrupt
        over_current_pin.make_interrupt_source(&mut afio);
        over_current_pin.trigger_on_edge(&exti, gpio::Edge::FALLING); // this has an external pullup
        over_current_pin.enable_interrupt(&exti);

        // setup over current interrupt
        fault_pin.make_interrupt_source(&mut afio);
        fault_pin.trigger_on_edge(&exti, gpio::Edge::FALLING); // this has an external pullup
        fault_pin.enable_interrupt(&exti);

        // take motor driver out of sleep mode
        sleep_pin.set_high().unwrap();

        // crate status leds
        let status1 = status::StatusLed::new_with_mode(
            gpiob.pb10.into_push_pull_output(&mut gpiob.crh),
            status::LedMode::FlashSlow,
        );
        let status2 = status::StatusLed::new_with_mode(
            gpiob.pb11.into_push_pull_output(&mut gpiob.crh),
            status::LedMode::FlashSlow,
        );

        // enable cycle counter
        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        // start periodic functions
        let now = cx.start;
        cx.schedule
            .led_update(now + LED_UPDATE_PD.cycles())
            .unwrap();
        cx.schedule
            .motor_update(now + MOTOR_UPDATE_PD.cycles())
            .unwrap();
        cx.schedule
            .can_tx(now + CAN_VALUE_UPDATE_PD.cycles())
            .unwrap();

        defmt::trace!("End of init");

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

    /// idle function. Does nothing for now
    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            // cortex_m::asm::wfi();
        }
    }

    #[task(capacity = 8, priority = 2, spawn = [queue_tx_frame], resources = [duty_now, current_now])]
    fn send_update(mut cx: send_update::Context) {
        defmt::trace!("Send update");

        // get resources
        let current_now = cx.resources.current_now.lock(|cn| *cn);
        let duty_now = cx.resources.duty_now.lock(|dn| *dn);

        // push an update frame to the queue
        let _ = cx
            .spawn
            .queue_tx_frame(can_types::OutgoingFrame::Update {
                current_now,
                duty_now,
            })
            .unwrap_or_else(|_| defmt::warn!("Could not queue frame"));

        // schedule this task again
        // cx.schedule
        // .send_update(Instant::now() + CAN_VALUE_UPDATE_PD.cycles())
        // .unwrap();
    }

    #[task(priority = 9, binds = EXTI9_5, spawn=[queue_tx_frame], resources = [over_current_pin, fault_pin, current_now, current_limit])]
    fn exti9_5(mut cx: exti9_5::Context) {
        defmt::trace!("Exti95");
        use can_types::IntoWithId;
        use can_types::OutgoingFrame;

        // get resources
        let oc_pin = cx.resources.over_current_pin;
        let f_pin = cx.resources.fault_pin;

        // check overcurrent interrupt
        if oc_pin.check_interrupt() {
            defmt::debug!("Overcurrent interrupt");

            // get nesessary variables
            let current_now = cx.resources.current_now.lock(|c| *c);
            let current_limit = cx.resources.current_limit.lock(|cl| *cl) as f32;

            // push overcurrent frame
            let _ = cx
                .spawn
                .queue_tx_frame(OutgoingFrame::Overcurrent {
                    current_now,
                    current_limit,
                })
                .unwrap_or_else(|_| defmt::warn!("Could not queue Frame"));

            oc_pin.clear_interrupt_pending_bit();
        }

        // check fault interrupt
        if f_pin.check_interrupt() {
            defmt::debug!("Motor fault interrupt");

            let _ = cx
                .spawn
                .queue_tx_frame(OutgoingFrame::Error(ErrorCode::MotorDriverFault {}))
                .unwrap_or_else(|_| defmt::warn!("Could not queue frame"));

            f_pin.clear_interrupt_pending_bit();
        }
    }

    /// motor update periodic task
    /// this runs at a high rate
    /// we set duty cycles and current limit here
    #[task(priority = 10, schedule = [motor_update], resources = [last_heartbeat, motor_low, motor_high, motor_current_limit, setpoint, current_limit, inverted, idle_mode, duty_now])]
    fn motor_update(cx: motor_update::Context) {
        defmt::trace!("MotorUpdate");
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

        let max_duty = motor_low.get_max_duty() as u16;

        #[cfg(feature = "heartbeat")]
        let mut stop = if let Some(last) = last_heartbeat {
            last.elapsed() > 0.cycles()
        } else {
            true
        };

        #[cfg(not(feature = "heartbeat"))]
        let mut stop = false;

        if setpoint.abs() < DEFAULT_MOTOR_DEADBAND {
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

        *cx.resources.duty_now = setpoint;

        // schedule this task again
        cx.schedule
            .motor_update(Instant::now() + MOTOR_UPDATE_PD.cycles())
            .unwrap();
    }

    #[task(priority = 10, binds = DMA1_CHANNEL1, resources=[adc_buf, current_now])]
    fn handle_adc(cx: handle_adc::Context) {
        defmt::trace!("Reading adc");

        let mut val: f32 = 0.0;

        match cx.resources.adc_buf.peek(|b, _| {
            val = b.iter().sum::<u16>() as f32 / ADC_BUF_LEN as f32;
        }) {
            Ok(_) => {}
            Err(_) => {
                defmt::warn!("DMA overrun");
                // let adc = unsafe { &*stm32f1xx_hal::pac::ADC1::ptr() };
                return;
            }
        }

        defmt::trace!("Actually reading dma");

        // We use floats here because the accuracy matters to an extent
        let volts = val / 1000.0;

        // this comes from the data sheet
        let current: f32 = ((volts - V_OFF) / (R_SENSE_VAL * AMP_GAIN)) / CURRENT_EXTERNAL_SCALE;

        defmt::info!("Motor current: {=f32}", current);

        *cx.resources.current_now = current;

        // NOTE:
        // we should really do something that takes the vref into account
        // such as: let x = adc_val * 1200 / adc.read_vref();
        // but i dont know how to get the vref value through the dma
        // for now we will just do it the dumb way
    }

    #[task(priority = 5, capacity = 32, resources=[can_tx_queue, can_id])]
    fn queue_tx_frame(cx: queue_tx_frame::Context, frame: can_types::OutgoingFrame) {
        use can_types::IntoWithId;

        let mut queue = cx.resources.can_tx_queue;
        let id = *cx.resources.can_id;

        match queue.push(allocate_tx_frame(frame.into_with_id(id))) {
            Ok(..) => {}
            Err(_) => {
                defmt::warn!("Can Tx queue is out of space");
            }
        }
    }

    #[task(priority = 5, capacity = 32, resources=[can_tx_queue, last_can_rx, inverted, current_limit, setpoint, last_heartbeat, idle_mode] )]
    fn handle_rx_frame(mut cx: handle_rx_frame::Context, frame: Frame) {
        use can_types::IncomingFrame;
        use can_types::IncomingFrame::*;
        use core::convert::TryFrom;

        let last_rx = cx.resources.last_can_rx;

        match IncomingFrame::try_from(frame) {
            Ok(Setpoint(setpoint)) => {
                defmt::info!("Setting setpoint to {=i16}", setpoint);
                cx.resources.setpoint.lock(|s| *s = setpoint);
            }
            Ok(SetCurrentLimit(limit)) => {
                defmt::info!("Setting current limit to {=u8} amps", limit);
                cx.resources.current_limit.lock(|cl| *cl = limit);
            }
            Ok(Invert(inv)) => {
                defmt::info!("Setting motor inversion to {=bool}", inv);
                cx.resources.inverted.lock(|inverted| *inverted = inv);
            }
            #[cfg(feature = "heartbeat")]
            Ok(HeartBeat) => {
                defmt::info!("Heartbeat...");
                cx.resources
                    .last_heartbeat
                    .lock(|lh| *lh = Some(Instant::now()));
            }
            #[cfg(not(feature = "heartbeat"))]
            Ok(HeartBeat) => {}
            Ok(Stop) => {
                defmt::info!("Stopping motor (setpoint = 0)");
                cx.resources.setpoint.lock(|s| *s = 0);
            }
            Ok(SetIdleMode(mode)) => {
                defmt::info!("Setting idle mode to {:?}", mode);
                cx.resources.idle_mode.lock(|im| *im = mode);
            }
            Err(e) => defmt::panic!("{:?}", e),
        };

        *last_rx = Some(Instant::now());
        rtic::pend(Interrupt::USB_HP_CAN_TX);
    }

    #[task(priority=5, binds = USB_LP_CAN_RX0, resources=[can_rx], spawn=[handle_rx_frame])]
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

    #[task(priority=5, binds = CAN_RX1)]
    fn can_rx1(_: can_rx1::Context) {
        rtic::pend(Interrupt::USB_LP_CAN_RX0);
    }

    #[task(priority = 4, schedule=[can_tx], spawn=[send_update], resources = [can_tx, can_tx_queue, last_can_rx])]
    fn can_tx(cx: can_tx::Context) {
        let tx = cx.resources.can_tx;
        let mut tx_queue = cx.resources.can_tx_queue;
        let mut last_rx = cx.resources.last_can_rx;

        let can_ok = if let Some(t) = last_rx.lock(|rx| *rx) {
            !(t.elapsed() > CAN_TIMEOUT.cycles())
        } else {
            false
        };

        tx.clear_interrupt_flags();

        if !can_ok {
            defmt::debug!("Canbus timeout, waiting for recieved frame before tx");
            cx.schedule
                .can_tx(Instant::now() + CAN_VALUE_UPDATE_PD.cycles())
                .unwrap();
        }

        // queue an update frame
        cx.spawn.send_update().unwrap();

        tx_queue.lock(|tx_queue| {
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
        });

        cx.schedule
            .can_tx(Instant::now() + CAN_VALUE_UPDATE_PD.cycles())
            .unwrap();
    }

    #[task(priority = 5, resources = [status1, status2], schedule=[led_update])]
    fn led_update(cx: led_update::Context) {
        defmt::trace!("Updating leds");
        let status1 = cx.resources.status1;
        let status2 = cx.resources.status2;

        status1.update();
        status2.update();

        cx.schedule
            .led_update(Instant::now() + LED_UPDATE_PD.cycles())
            .unwrap();
    }

    #[allow(non_snake_case)]
    extern "C" {
        fn USART1();
        fn USART2();

        fn UART4();
        fn UART5();

        fn ADC3();

        fn SPI2();
        fn SPI3();

        fn EXTI1();
        fn EXTI2();
    }
};

#[panic_handler]
#[inline(never)]
fn my_panic(info: &core::panic::PanicInfo) -> ! {
    use defmt::Debug2Format;

    defmt::error!(
        "Panic: \"{:?}\" \nin file {=str} at line {=u32}",
        Debug2Format(info.message().unwrap()),
        info.location().unwrap().file(),
        info.location().unwrap().line()
    );
    loop {
        core::hint::spin_loop();
    }
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

#[lang = "eh_personality"]
extern "C" fn eh_personality() {}
