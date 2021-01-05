#![feature(lang_items, panic_info_message, fmt_as_str)]
#![no_std]
#![no_main]
#![allow(dead_code)]

use core::prelude::*;
use embedded_hal::prelude::*;
use stm32f1xx_hal::prelude::*;

use embedded_hal::digital::v2::{InputPin, OutputPin};

use cortex_m::peripheral::DWT;

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

use stm32f1xx_hal::pac::{Interrupt, CAN1};
use stm32f1xx_hal::{can, gpio, pac};

use bxcan::{filter::Mask32, Can, Frame, Id, Rx, StandardId, Tx};

use heapless;
use heapless::BinaryHeap;

use core::cmp::Ordering;
use core::convert::Into;

use defmt;

/// this makes sure that the rtt logger is linked into the binary
use defmt_rtt as _;

pub struct PriorityFrame(Frame);

impl Ord for PriorityFrame {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.priority().cmp(&other.0.priority())
    }
}

impl PartialOrd for PriorityFrame {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Eq for PriorityFrame {}
impl PartialEq for PriorityFrame {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

const HSE_CLOCK_MHZ: u32 = 8;
const SYS_CLOCK_MHZ: u32 = 72;

const HSE_CLOCK_HZ: u32 = HSE_CLOCK_MHZ * 1_000_000;
const SYS_CLOCK_HZ: u32 = SYS_CLOCK_MHZ * 1_000_000;

const CAN_CONFIG: u32 = 0x001e0001;
const CAN_TIMEOUT: u32 = SYS_CLOCK_MHZ * 2;

#[defmt::timestamp]
fn timestamp() -> u64 {
    DWT::get_cycle_count() as u64
}

heapless::pool! {
    #[allow(non_upper_case_globals)]
    CanFramePool: PriorityFrame
}

fn allocate_tx_frame(
    frame: Frame,
) -> heapless::pool::singleton::Box<CanFramePool, heapless::pool::Init> {
    use heapless::pool::singleton::Pool;
    let b = CanFramePool::alloc().unwrap();
    b.init(PriorityFrame(frame))
}

#[app(device=stm32f1::stm32f103, peripherals = true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(None)]
        last_can_rx: Option<Instant>,
        can_id: bxcan::StandardId,
        can_tx: Tx<can::Can<pac::CAN1>>,
        can_rx: Rx<can::Can<pac::CAN1>>,

        can_tx_queue: heapless::BinaryHeap<
            heapless::pool::singleton::Box<CanFramePool, heapless::pool::Init>,
            heapless::consts::U16,
            heapless::binary_heap::Max,
        >,
    }

    #[init(schedule=[])]
    fn init(cx: init::Context) -> init::LateResources {
        static mut CAN_TX_MEM: [u8; 1024] = [0; 1024];

        let can_tx_queue = BinaryHeap::new();

        use heapless::pool::singleton::Pool;
        CanFramePool::grow(CAN_TX_MEM);

        let mut peripherals = cx.core;
        let device: stm32f1xx_hal::stm32::Peripherals = cx.device;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

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

        #[allow(unused_must_use)]
        let can_id = {
            let mut id = 0_u16;
            let mut pins =
                heapless::Vec::<gpio::Pxx<gpio::Input<gpio::PullDown>>, heapless::consts::U8>::new(
                );
            // put all can_id pins in a vec
            pins.push(gpiob.pb14.into_pull_down_input(&mut gpiob.crh).downgrade());
            pins.push(gpiob.pb13.into_pull_down_input(&mut gpiob.crh).downgrade());
            pins.push(gpiob.pb12.into_pull_down_input(&mut gpiob.crh).downgrade());
            pins.push(gpiob.pb2.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb1.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpiob.pb0.into_pull_down_input(&mut gpiob.crl).downgrade());
            pins.push(gpioa.pa4.into_pull_down_input(&mut gpioa.crl).downgrade());
            pins.push(gpioa.pa3.into_pull_down_input(&mut gpioa.crl).downgrade());

            // interate over the pins and shift values as we go
            for (shift, pin) in pins.iter().enumerate() {
                id |= (pin.is_high().unwrap() as u16) << (shift as u16);
                defmt::debug!("Id so far: {:u16}", id);
            }
            pins.clear(); // just to make sure this happens.
            id
        };

        let can_id = StandardId::new(can_id).unwrap();

        let can = can::Can::new(device.CAN1, &mut rcc.apb1, device.USB);

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

        nb::block!(can.enable()).unwrap();

        let (can_tx, can_rx) = can.split();

        peripherals.DCB.enable_trace();
        DWT::unlock();
        peripherals.DWT.enable_cycle_counter();

        // let now = cx.start;
        //
        init::LateResources {
            can_id,
            can_tx_queue,
            can_tx,
            can_rx,
        }
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            core::sync::atomic::spin_loop_hint();
        }
    }

    #[task(capacity = 16, resources=[can_id, can_tx_queue, last_can_rx] )]
    fn handle_rx_frame(cx: handle_rx_frame::Context, frame: Frame) {
        let can_id = cx.resources.can_id;
        let mut last_rx = cx.resources.last_can_rx;
        let mut tx_queue = cx.resources.can_tx_queue;

        let rx_id = match frame.id() {
            bxcan::Id::Standard(id) => id.as_raw(),
            bxcan::Id::Extended(_) => return,
        };

        let _id = rx_id & 0xFF;
        let _cmd = rx_id << 8;

        if frame.is_remote_frame() {
            let ret_frame = Frame::new_data(bxcan::Id::Standard(*can_id), (0_u16).to_ne_bytes());
            tx_queue.lock(|q| {
                #[allow(unused_must_use)]
                q.push(allocate_tx_frame(ret_frame));
            })
        } else {
            defmt::todo!("Implement these reponses");
        }

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
                    #[allow(unused_must_use)]
                    tx_queue.push(allocate_tx_frame(pending_frame));
                }
                Err(nb::Error::WouldBlock) => break,
                Err(_) => unreachable!(),
            }
        }
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
