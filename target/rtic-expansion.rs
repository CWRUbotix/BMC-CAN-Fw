#[allow(non_snake_case)] fn init(cx : init :: Context) -> init ::
LateResources
{
    let can_tx_queue = BinaryHeap :: new() ; CanFramePool ::
    grow(cortex_m :: singleton ! (: [u8 ; 1024] = [0 ; 1024]) . unwrap()) ;
    let mut peripherals = cx . core ; let device : stm32f1xx_hal :: stm32 ::
    Peripherals = cx . device ; let mut flash = device . FLASH . constrain() ;
    let mut rcc = device . RCC . constrain() ; let mut afio = device . AFIO .
    constrain(& mut rcc . apb2) ; let mut debug = device . DBGMCU ; let clocks
    = rcc . cfgr . use_hse(HSE_CLOCK_MHZ . mhz()) .
    sysclk(SYS_CLOCK_MHZ . mhz()) . hclk(SYS_CLOCK_MHZ . mhz()) .
    pclk1((SYS_CLOCK_MHZ / 2) . mhz()) . pclk2(SYS_CLOCK_MHZ . mhz()) .
    freeze(& mut flash . acr) ; let mut gpioa = device . GPIOA .
    split(& mut rcc . apb2) ; let mut gpiob = device . GPIOB .
    split(& mut rcc . apb2) ; #[allow(unused_must_use)] let can_id =
    {
        let mut id = 0_u16 ; let mut pins = heapless :: Vec :: < gpio :: Pxx <
        gpio :: Input < gpio :: PullDown > >, consts :: U8 > :: new() ;
        let(_, pb3, pb4) = afio . mapr .
        disable_jtag(gpioa . pa15, gpiob . pb3, gpiob . pb4) ; pins .
        push(gpiob . pb0 . into_pull_down_input(& mut gpiob . crl) .
             downgrade()) ; pins .
        push(pb3 . into_pull_down_input(& mut gpiob . crl) . downgrade()) ;
        pins .
        push(pb4 . into_pull_down_input(& mut gpiob . crl) . downgrade()) ;
        pins .
        push(gpiob . pb5 . into_pull_down_input(& mut gpiob . crl) .
             downgrade()) ; pins .
        push(gpiob . pb6 . into_pull_down_input(& mut gpiob . crl) .
             downgrade()) ; pins .
        push(gpiob . pb7 . into_pull_down_input(& mut gpiob . crl) .
             downgrade()) ; for(shift, pin) in pins . iter() . enumerate()
        {
            id |= (pin . is_high() . unwrap() as u16) << (shift as u16) ;
            defmt :: debug ! ("Id so far: {:u16}", id) ;
        } pins . clear() ; id
    } ; let can_id = StandardId :: new(can_id) . unwrap() ; let can = can ::
    Can :: new(device . CAN1, & mut rcc . apb1, device . USB) ;
    {
        let tx_pin = gpiob . pb9 . into_alternate_push_pull(& mut gpiob . crh)
        ; let rx_pin = gpiob . pb8 . into_floating_input(& mut gpiob . crh) ;
        can . assign_pins((tx_pin, rx_pin), & mut afio . mapr) ;
    } let mut can = bxcan :: Can :: new(can) ; can .
    configure(| config |
              {
                  config . set_bit_timing(CAN_CONFIG) ; config .
                  set_silent(false) ; config . set_loopback(false) ;
              }) ; let can_id_mask = StandardId :: new(0xFF) . unwrap() ; let
    mut can_filters = can . modify_filters() ; can_filters .
    enable_bank(0, Mask32 :: frames_with_std_id(can_id, can_id_mask)) ;
    drop(can_filters) ; let(_, mut c2, mut c3, mut c4) =
    {
        let tim_pins =
        (gpioa . pa8 . into_alternate_push_pull(& mut gpioa . crh), gpioa .
         pa9 . into_alternate_push_pull(& mut gpioa . crh), gpioa . pa10 .
         into_alternate_push_pull(& mut gpioa . crh), gpioa . pa11 .
         into_alternate_push_pull(& mut gpioa . crh),) ; let mut tim =
        stm32f1xx_hal :: timer :: Timer ::
        tim1(device . TIM1, & clocks, & mut rcc . apb2) ; tim .
        stop_in_debug(& mut debug, false) ; tim . pwm :: < stm32f1xx_hal ::
        timer :: Tim1NoRemap, _, _, _ >
        (tim_pins, & mut afio . mapr, 1 . khz()) . split()
    } ; c2 . enable() ; c3 . enable() ; c4 . enable() ; let adc_buf =
    {
        let dma_ch = device . DMA1 . split(& mut rcc . ahb) . 1 ; let ch0 =
        gpioa . pa3 . into_analog(& mut gpioa . crl) ; let mut adc =
        stm32f1xx_hal :: adc :: Adc ::
        adc1(device . ADC1, & mut rcc . apb2, clocks) ; adc .
        set_continuous_mode(true) ; adc .
        set_sample_time(stm32f1xx_hal :: adc :: SampleTime :: T_13) ; let buf
        = cortex_m :: singleton ! (: [u16 ; 2] = [0 ; 2]) . unwrap() ; adc .
        with_dma(ch0, dma_ch) . circ_read(buf)
    } ; nb :: block ! (can . enable()) . unwrap() ; let(can_tx, can_rx) = can
    . split() ; peripherals . DCB . enable_trace() ; DWT :: unlock() ;
    peripherals . DWT . enable_cycle_counter() ; init :: LateResources
    { can_id, can_tx_queue, can_tx, can_rx, adc_buf, }
} #[allow(non_snake_case)] fn idle(_cx : idle :: Context) -> !
{
    use rtic :: Mutex as _ ; loop
    { core :: sync :: atomic :: spin_loop_hint() ; }
} #[allow(non_snake_case)] fn can_rx0(cx : can_rx0 :: Context)
{
    use rtic :: Mutex as _ ; let rx = cx . resources . can_rx ; loop
    {
        match rx . receive()
        {
            Ok(frame) => { cx . spawn . handle_rx_frame(frame) . unwrap() ; }
            Err(nb :: Error :: WouldBlock) => break,
            Err(nb :: Error :: Other(_)) => { }
        }
    }
} #[allow(non_snake_case)] fn can_rx1(_ : can_rx1 :: Context)
{ use rtic :: Mutex as _ ; rtic :: pend(Interrupt :: USB_LP_CAN_RX0) ; }
#[allow(non_snake_case)] fn can_tx(cx : can_tx :: Context)
{
    use rtic :: Mutex as _ ; let tx = cx . resources . can_tx ; let tx_queue =
    cx . resources . can_tx_queue ; let last_rx = cx . resources . last_can_rx
    ; let can_ok = if let Some(t) = last_rx
    { ! (t . elapsed() > CAN_TIMEOUT . cycles()) } else { false } ; tx .
    clear_interrupt_flags() ; if ! can_ok
    {
        defmt :: debug !
        ("Canbus timeout, waiting for recieved frame before tx") ; return ;
    } while let Some(frame) = tx_queue . peek()
    {
        match tx . transmit(& frame . 0)
        {
            Ok(None) =>
            {
                use core :: ops :: Deref ; let sent_frame = tx_queue . pop() ;
                defmt :: info !
                ("Sent Frame: {:?}", sent_frame . unwrap() . deref() . 0) ;
            } Ok(Some(pending_frame)) =>
            {
                tx_queue . pop() ; #[allow(unused_must_use)] tx_queue .
                push(allocate_tx_frame(pending_frame)) ;
            } Err(nb :: Error :: WouldBlock) => break, Err(_) => unreachable !
            (),
        }
    }
} #[allow(non_snake_case)] fn
handle_rx_frame(cx : handle_rx_frame :: Context, frame : Frame)
{
    use rtic :: Mutex as _ ; let can_id = cx . resources . can_id ; let mut
    last_rx = cx . resources . last_can_rx ; let mut tx_queue = cx . resources
    . can_tx_queue ; let rx_id = match frame . id()
    {
        bxcan :: Id :: Standard(id) => id . as_raw(), bxcan :: Id ::
        Extended(_) => return,
    } ; let _id = rx_id & 0xFF ; let _cmd = rx_id << 8 ; if frame .
    is_remote_frame()
    {
        let ret_frame = Frame ::
        new_data(bxcan :: Id :: Standard(* can_id), (0_u16) . to_ne_bytes()) ;
        tx_queue .
        lock(| q |
             {
                 #[allow(unused_must_use)] q .
                 push(allocate_tx_frame(ret_frame)) ;
             })
    } else { defmt :: todo ! ("Implement these reponses") ; } last_rx .
    lock(| instant : & mut Option < Instant > | * instant =
         Some(Instant :: now())) ; rtic :: pend(Interrupt :: USB_HP_CAN_TX) ;
} #[doc = r" Resources initialized at runtime"] #[allow(non_snake_case)] pub
struct initLateResources
{
    pub adc_buf : dma :: CircBuffer < u16, AdcDma >, pub can_id : bxcan ::
    StandardId, pub can_rx : Rx < can :: Can < pac :: CAN1 > >, pub can_tx :
    Tx < can :: Can < pac :: CAN1 > >, pub can_tx_queue : heapless ::
    BinaryHeap < Box < CanFramePool, heapless :: pool :: Init >, consts ::
    U16, heapless :: binary_heap :: Max, >
} #[allow(non_snake_case)] #[doc = "Initialization function"] pub mod init
{
    #[doc(inline)] pub use super :: initLateResources as LateResources ;
    #[doc = r" Execution context"] pub struct Context < >
    {
        #[doc = r" Core (Cortex-M) peripherals"] pub core : rtic :: export ::
        Peripherals, #[doc = r" Device peripherals"] pub device : stm32f1 ::
        stm32f103 :: Peripherals,
    } impl < > Context < >
    {
        #[inline(always)] pub unsafe fn
        new(core : rtic :: export :: Peripherals,) -> Self
        {
            Context
            { device : stm32f1 :: stm32f103 :: Peripherals :: steal(), core, }
        }
    }
} #[allow(non_snake_case)] #[doc = "Idle loop"] pub mod idle
{
    #[doc = r" Execution context"] pub struct Context < > { } impl < > Context
    < >
    {
        #[inline(always)] pub unsafe fn
        new(priority : & rtic :: export :: Priority) -> Self { Context { } }
    }
} mod resources
{
    use rtic :: export :: Priority ; #[allow(non_camel_case_types)] pub struct
    can_tx_queue < 'a > { priority : & 'a Priority, } impl < 'a > can_tx_queue
    < 'a >
    {
        #[inline(always)] pub unsafe fn new(priority : & 'a Priority) -> Self
        { can_tx_queue { priority } } #[inline(always)] pub unsafe fn
        priority(& self) -> & Priority { self . priority }
    } #[allow(non_camel_case_types)] pub struct last_can_rx < 'a >
    { priority : & 'a Priority, } impl < 'a > last_can_rx < 'a >
    {
        #[inline(always)] pub unsafe fn new(priority : & 'a Priority) -> Self
        { last_can_rx { priority } } #[inline(always)] pub unsafe fn
        priority(& self) -> & Priority { self . priority }
    }
} #[allow(non_snake_case)] #[doc = "Resources `can_rx0` has access to"] pub
struct can_rx0Resources < 'a >
{ pub can_rx : & 'a mut Rx < can :: Can < pac :: CAN1 > >, }
#[allow(non_snake_case)] #[doc = "Hardware task"] pub mod can_rx0
{
    #[doc(inline)] pub use super :: can_rx0Resources as Resources ;
    #[doc = r" Tasks that can be spawned from this context"]
    #[derive(Clone, Copy)] pub struct Spawn < 'a >
    { priority : & 'a rtic :: export :: Priority, } impl < 'a > Spawn < 'a >
    {
        #[doc(hidden)] #[inline(always)] pub unsafe fn priority(& self) -> &
        rtic :: export :: Priority { self . priority }
    } #[doc = r" Execution context"] pub struct Context < 'a >
    {
        #[doc = r" Resources this task has access to"] pub resources :
        Resources < 'a >,
        #[doc = "Tasks that can be `spawn`-ed from this context"] pub spawn :
        Spawn < 'a >,
    } impl < 'a > Context < 'a >
    {
        #[inline(always)] pub unsafe fn
        new(priority : & 'a rtic :: export :: Priority) -> Self
        {
            Context
            {
                resources : Resources :: new(priority), spawn : Spawn
                { priority },
            }
        }
    }
} #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod can_rx1
{
    #[doc = r" Execution context"] pub struct Context < > { } impl < > Context
    < >
    {
        #[inline(always)] pub unsafe fn
        new(priority : & rtic :: export :: Priority) -> Self { Context { } }
    }
} #[allow(non_snake_case)] #[doc = "Resources `can_tx` has access to"] pub
struct can_txResources < 'a >
{
    pub can_tx : & 'a mut Tx < can :: Can < pac :: CAN1 > >, pub can_tx_queue
    : & 'a mut heapless :: BinaryHeap < Box < CanFramePool, heapless :: pool
    :: Init >, consts :: U16, heapless :: binary_heap :: Max, >, pub
    last_can_rx : & 'a mut Option < Instant >,
} #[allow(non_snake_case)] #[doc = "Hardware task"] pub mod can_tx
{
    #[doc(inline)] pub use super :: can_txResources as Resources ;
    #[doc = r" Execution context"] pub struct Context < 'a >
    {
        #[doc = r" Resources this task has access to"] pub resources :
        Resources < 'a >,
    } impl < 'a > Context < 'a >
    {
        #[inline(always)] pub unsafe fn
        new(priority : & 'a rtic :: export :: Priority) -> Self
        { Context { resources : Resources :: new(priority), } }
    }
} #[allow(non_snake_case)]
#[doc = "Resources `handle_rx_frame` has access to"] pub struct
handle_rx_frameResources < 'a >
{
    pub can_id : & 'a mut bxcan :: StandardId, pub can_tx_queue : resources ::
    can_tx_queue < 'a >, pub last_can_rx : resources :: last_can_rx < 'a >,
} #[allow(non_snake_case)] #[doc = "Software task"] pub mod handle_rx_frame
{
    #[doc(inline)] pub use super :: handle_rx_frameResources as Resources ;
    #[doc = r" Execution context"] pub struct Context < 'a >
    {
        #[doc = r" Resources this task has access to"] pub resources :
        Resources < 'a >,
    } impl < 'a > Context < 'a >
    {
        #[inline(always)] pub unsafe fn
        new(priority : & 'a rtic :: export :: Priority) -> Self
        { Context { resources : Resources :: new(priority), } }
    }
} #[doc = r" Implementation details"] const APP : () =
{
    #[doc =
      r" Always include the device crate which contains the vector table"] use
    stm32f1 :: stm32f103 as _ ; #[cfg(core = "1")] compile_error !
    ("specified 1 core but tried to compile for more than 1 core") ;
    #[allow(non_upper_case_globals)] #[link_section = ".uninit.rtic0"] static
    mut can_rx : core :: mem :: MaybeUninit < Rx < can :: Can < pac :: CAN1 >
    > > = core :: mem :: MaybeUninit :: uninit() ;
    #[allow(non_upper_case_globals)] #[link_section = ".uninit.rtic1"] static
    mut can_tx : core :: mem :: MaybeUninit < Tx < can :: Can < pac :: CAN1 >
    > > = core :: mem :: MaybeUninit :: uninit() ;
    #[allow(non_upper_case_globals)] #[link_section = ".uninit.rtic2"] static
    mut can_tx_queue : core :: mem :: MaybeUninit < heapless :: BinaryHeap <
    Box < CanFramePool, heapless :: pool :: Init >, consts :: U16, heapless ::
    binary_heap :: Max, > > = core :: mem :: MaybeUninit :: uninit() ; impl <
    'a > rtic :: Mutex for resources :: can_tx_queue < 'a >
    {
        type T = heapless :: BinaryHeap < Box < CanFramePool, heapless :: pool
        :: Init >, consts :: U16, heapless :: binary_heap :: Max, > ;
        #[inline(always)] fn lock < R >
        (& mut self, f : impl
         FnOnce(& mut heapless :: BinaryHeap < Box < CanFramePool, heapless ::
                pool :: Init >, consts :: U16, heapless :: binary_heap :: Max,
                >) -> R) -> R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8 ; unsafe
            {
                rtic :: export ::
                lock(can_tx_queue . as_mut_ptr(), self . priority(), CEILING,
                     stm32f1 :: stm32f103 :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_upper_case_globals)] #[doc = " all the CAN bus resources"]
    static mut last_can_rx : Option < Instant > = None ; impl < 'a > rtic ::
    Mutex for resources :: last_can_rx < 'a >
    {
        type T = Option < Instant > ; #[inline(always)] fn lock < R >
        (& mut self, f : impl FnOnce(& mut Option < Instant >) -> R) -> R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8 ; unsafe
            {
                rtic :: export ::
                lock(& mut last_can_rx, self . priority(), CEILING, stm32f1 ::
                     stm32f103 :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_upper_case_globals)] #[link_section = ".uninit.rtic3"]
    static mut can_id : core :: mem :: MaybeUninit < bxcan :: StandardId > =
    core :: mem :: MaybeUninit :: uninit() ; #[allow(non_snake_case)]
    #[no_mangle] unsafe fn USB_LP_CAN_RX0()
    {
        const PRIORITY : u8 = 2u8 ; rtic :: export ::
        run(PRIORITY, ||
            {
                crate ::
                can_rx0(can_rx0 :: Context ::
                        new(& rtic :: export :: Priority :: new(PRIORITY)))
            }) ;
    } impl < 'a > can_rx0Resources < 'a >
    {
        #[inline(always)] unsafe fn
        new(priority : & 'a rtic :: export :: Priority) -> Self
        { can_rx0Resources { can_rx : & mut * can_rx . as_mut_ptr(), } }
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn CAN_RX1()
    {
        const PRIORITY : u8 = 2u8 ; rtic :: export ::
        run(PRIORITY, ||
            {
                crate ::
                can_rx1(can_rx1 :: Context ::
                        new(& rtic :: export :: Priority :: new(PRIORITY)))
            }) ;
    } #[allow(non_snake_case)] #[no_mangle] unsafe fn USB_HP_CAN_TX()
    {
        const PRIORITY : u8 = 2u8 ; rtic :: export ::
        run(PRIORITY, ||
            {
                crate ::
                can_tx(can_tx :: Context ::
                       new(& rtic :: export :: Priority :: new(PRIORITY)))
            }) ;
    } impl < 'a > can_txResources < 'a >
    {
        #[inline(always)] unsafe fn
        new(priority : & 'a rtic :: export :: Priority) -> Self
        {
            can_txResources
            {
                can_tx : & mut * can_tx . as_mut_ptr(), can_tx_queue : & mut *
                can_tx_queue . as_mut_ptr(), last_can_rx : & mut last_can_rx,
            }
        }
    }
    #[doc =
      r" Queue version of a free-list that keeps track of empty slots in"]
    #[doc = r" the following buffers"] static mut handle_rx_frame_S0_FQ : rtic
    :: export :: SCFQ < rtic :: export :: consts :: U16 > = rtic :: export ::
    Queue(unsafe { rtic :: export :: iQueue :: u8_sc() }) ; struct
    handle_rx_frame_S0_FQ < 'a >
    { priority : & 'a rtic :: export :: Priority, } impl < 'a > rtic :: Mutex
    for handle_rx_frame_S0_FQ < 'a >
    {
        type T = rtic :: export :: SCFQ < rtic :: export :: consts :: U16 > ;
        #[inline(always)] fn lock < R >
        (& mut self, f : impl
         FnOnce(& mut rtic :: export :: SCFQ < rtic :: export :: consts :: U16
                >) -> R) -> R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8 ; unsafe
            {
                rtic :: export ::
                lock(& mut handle_rx_frame_S0_FQ, self . priority, CEILING,
                     stm32f1 :: stm32f103 :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[link_section = ".uninit.rtic4"]
    #[doc = r" Buffer that holds the inputs of a task"] static mut
    handle_rx_frame_S0_INPUTS : [core :: mem :: MaybeUninit < Frame > ; 16] =
    [core :: mem :: MaybeUninit :: uninit(), core :: mem :: MaybeUninit ::
     uninit(), core :: mem :: MaybeUninit :: uninit(), core :: mem ::
     MaybeUninit :: uninit(), core :: mem :: MaybeUninit :: uninit(), core ::
     mem :: MaybeUninit :: uninit(), core :: mem :: MaybeUninit :: uninit(),
     core :: mem :: MaybeUninit :: uninit(), core :: mem :: MaybeUninit ::
     uninit(), core :: mem :: MaybeUninit :: uninit(), core :: mem ::
     MaybeUninit :: uninit(), core :: mem :: MaybeUninit :: uninit(), core ::
     mem :: MaybeUninit :: uninit(), core :: mem :: MaybeUninit :: uninit(),
     core :: mem :: MaybeUninit :: uninit(), core :: mem :: MaybeUninit ::
     uninit(),] ; impl < 'a > handle_rx_frameResources < 'a >
    {
        #[inline(always)] unsafe fn
        new(priority : & 'a rtic :: export :: Priority) -> Self
        {
            handle_rx_frameResources
            {
                can_id : & mut * can_id . as_mut_ptr(), can_tx_queue :
                resources :: can_tx_queue :: new(priority), last_can_rx :
                resources :: last_can_rx :: new(priority),
            }
        }
    } #[allow(non_camel_case_types)] #[derive(Clone, Copy)]
    #[doc =
      "Software tasks spawned from core #0 to be dispatched at priority level 1 by core #0"]
    enum R0_P1_S0_T { handle_rx_frame, }
    #[doc =
      "Queue of tasks sent by core #0 ready to be dispatched by core #0 at priority level 1"]
    static mut R0_P1_S0_RQ : rtic :: export :: SCRQ < R0_P1_S0_T, rtic ::
    export :: consts :: U16 > = rtic :: export ::
    Queue(unsafe { rtic :: export :: iQueue :: u8_sc() }) ; struct R0_P1_S0_RQ
    < 'a > { priority : & 'a rtic :: export :: Priority, } impl < 'a > rtic ::
    Mutex for R0_P1_S0_RQ < 'a >
    {
        type T = rtic :: export :: SCRQ < R0_P1_S0_T, rtic :: export :: consts
        :: U16 > ; #[inline(always)] fn lock < R >
        (& mut self, f : impl
         FnOnce(& mut rtic :: export :: SCRQ < R0_P1_S0_T, rtic :: export ::
                consts :: U16 >) -> R) -> R
        {
            #[doc = r" Priority ceiling"] const CEILING : u8 = 2u8 ; unsafe
            {
                rtic :: export ::
                lock(& mut R0_P1_S0_RQ, self . priority, CEILING, stm32f1 ::
                     stm32f103 :: NVIC_PRIO_BITS, f,)
            }
        }
    } #[allow(non_snake_case)]
    #[doc =
      "Interrupt handler used by core #0 to dispatch tasks at priority 1"]
    #[no_mangle] unsafe fn USART2()
    {
        #[doc = r" The priority of this interrupt handler"] const PRIORITY :
        u8 = 1u8 ; rtic :: export ::
        run(PRIORITY, ||
            {
                while let Some((task, index)) = R0_P1_S0_RQ . split() . 1 .
                dequeue()
                {
                    match task
                    {
                        R0_P1_S0_T :: handle_rx_frame =>
                        {
                            let _0 = handle_rx_frame_S0_INPUTS .
                            get_unchecked(usize :: from(index)) . as_ptr() .
                            read() ; handle_rx_frame_S0_FQ . split() . 0 .
                            enqueue_unchecked(index) ; let priority = & rtic
                            :: export :: Priority :: new(PRIORITY) ; crate ::
                            handle_rx_frame(handle_rx_frame :: Context ::
                                            new(priority), _0)
                        }
                    }
                }
            }) ;
    } unsafe fn
    spawn_handle_rx_frame_S0(priority : & rtic :: export :: Priority, _0 :
                             Frame) -> Result < (), Frame >
    {
        unsafe
        {
            use rtic :: Mutex as _ ; let input = _0 ; if let Some(index) =
            (handle_rx_frame_S0_FQ { priority } .
             lock(| fq | fq . split() . 1 . dequeue()))
            {
                handle_rx_frame_S0_INPUTS .
                get_unchecked_mut(usize :: from(index)) . as_mut_ptr() .
                write(input) ;
                (R0_P1_S0_RQ { priority } .
                 lock(| rq |
                      {
                          rq . split() . 0 .
                          enqueue_unchecked((R0_P1_S0_T :: handle_rx_frame,
                                             index))
                      })) ; rtic ::
                pend(stm32f1 :: stm32f103 :: Interrupt :: USART2) ; Ok(())
            } else { Err(input) }
        }
    } impl < 'a > can_rx0 :: Spawn < 'a >
    {
        #[inline(always)] fn handle_rx_frame(& self, _0 : Frame) -> Result <
        (), Frame >
        { unsafe { spawn_handle_rx_frame_S0(self . priority(), _0) } }
    } #[no_mangle] unsafe extern "C" fn main() -> !
    {
        let _TODO : () = () ; rtic :: export :: assert_send :: < bxcan ::
        StandardId > () ; rtic :: export :: assert_send :: < Tx < can :: Can <
        pac :: CAN1 > > > () ; rtic :: export :: assert_send :: < Rx < can ::
        Can < pac :: CAN1 > > > () ; rtic :: export :: assert_send :: <
        heapless :: BinaryHeap < Box < CanFramePool, heapless :: pool :: Init
        >, consts :: U16, heapless :: binary_heap :: Max, > > () ; rtic ::
        export :: assert_send :: < Frame > () ; rtic :: export :: interrupt ::
        disable() ; (0 .. 16u8) .
        for_each(| i | handle_rx_frame_S0_FQ . enqueue_unchecked(i)) ; let mut
        core : rtic :: export :: Peripherals = rtic :: export :: Peripherals
        :: steal() . into() ; let _ =
        [() ; ((1 << stm32f1 :: stm32f103 :: NVIC_PRIO_BITS) - 1u8 as usize)]
        ; core . NVIC .
        set_priority(stm32f1 :: stm32f103 :: Interrupt :: USART2, rtic ::
                     export ::
                     logical2hw(1u8, stm32f1 :: stm32f103 :: NVIC_PRIO_BITS),)
        ; rtic :: export :: NVIC ::
        unmask(stm32f1 :: stm32f103 :: Interrupt :: USART2) ; let _ =
        [() ; ((1 << stm32f1 :: stm32f103 :: NVIC_PRIO_BITS) - 2u8 as usize)]
        ; core . NVIC .
        set_priority(stm32f1 :: stm32f103 :: Interrupt :: USB_LP_CAN_RX0, rtic
                     :: export ::
                     logical2hw(2u8, stm32f1 :: stm32f103 :: NVIC_PRIO_BITS),)
        ; rtic :: export :: NVIC ::
        unmask(stm32f1 :: stm32f103 :: Interrupt :: USB_LP_CAN_RX0) ; let _ =
        [() ; ((1 << stm32f1 :: stm32f103 :: NVIC_PRIO_BITS) - 2u8 as usize)]
        ; core . NVIC .
        set_priority(stm32f1 :: stm32f103 :: Interrupt :: CAN_RX1, rtic ::
                     export ::
                     logical2hw(2u8, stm32f1 :: stm32f103 :: NVIC_PRIO_BITS),)
        ; rtic :: export :: NVIC ::
        unmask(stm32f1 :: stm32f103 :: Interrupt :: CAN_RX1) ; let _ =
        [() ; ((1 << stm32f1 :: stm32f103 :: NVIC_PRIO_BITS) - 2u8 as usize)]
        ; core . NVIC .
        set_priority(stm32f1 :: stm32f103 :: Interrupt :: USB_HP_CAN_TX, rtic
                     :: export ::
                     logical2hw(2u8, stm32f1 :: stm32f103 :: NVIC_PRIO_BITS),)
        ; rtic :: export :: NVIC ::
        unmask(stm32f1 :: stm32f103 :: Interrupt :: USB_HP_CAN_TX) ; let late
        = crate :: init(init :: Context :: new(core . into())) ; can_id .
        as_mut_ptr() . write(late . can_id) ; can_rx . as_mut_ptr() .
        write(late . can_rx) ; can_tx . as_mut_ptr() . write(late . can_tx) ;
        can_tx_queue . as_mut_ptr() . write(late . can_tx_queue) ; rtic ::
        export :: interrupt :: enable() ; crate ::
        idle(idle :: Context :: new(& rtic :: export :: Priority :: new(0)))
    }
} ;