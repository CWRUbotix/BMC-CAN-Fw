use bxcan::{Data, ExtendedId, Frame, Id, StandardId};

use core::cmp::Ordering;

use core::convert::{Into, TryFrom};

use defmt::Format;

pub enum FrameConversionError {
    FrameType,
    InvalidFrame(&'static str),
    TooShort { minimum: u8, actual: u8 },
    InvalidIdFormat,
    InvalidCommand { cmd: u16 },
}

impl Format for FrameConversionError {
    fn format(&self, f: &mut defmt::Formatter) {
        use FrameConversionError::*;
        match self {
            FrameType => {
                defmt::write!(f, "Frame type is incorrect (remote vs data frames)");
            }
            InvalidFrame(s) => {
                defmt::write!(f, "Invalid Frame: {:str}", s);
            }
            TooShort { minimum, actual } => {
                defmt::write!(
                    f,
                    "Data frame too short. Recieved dlc = {:u8}, requires length {:u8}",
                    actual,
                    minimum
                );
            }
            InvalidIdFormat => {
                defmt::write!(f, "Invalid id format");
            }
            InvalidCommand { cmd } => {
                defmt::write!(f, "Invalid command {:u16}", cmd);
            }
        }
    }
}

#[derive(Format)]
pub enum IncomingFrame {
    Setpoint(i16),
    SetCurrentLimit(u8),
    Invert(bool),
    HeartBeat,
    Stop,
}

macro_rules! check_frame_size {
    ($req:expr, $actual:expr) => {
        if $actual < $req {
            return Err(FrameConversionError::TooShort {
                actual: $actual,
                minimum: $req,
            });
        }
    };
}

impl TryFrom<Frame> for IncomingFrame {
    type Error = FrameConversionError;
    fn try_from(frame: Frame) -> Result<IncomingFrame, Self::Error> {
        let dlc = frame.dlc();

        let rx_id = match frame.id() {
            bxcan::Id::Standard(id) => id.as_raw(),
            _ => return Err(FrameConversionError::InvalidIdFormat),
        };

        // NOTE: The id should already be correct since we have filters
        let _id = rx_id & 0xFF;
        let cmd = (rx_id >> 8) & 0xFF;

        if frame.is_remote_frame() {
            if cmd == 0 {
                Ok(IncomingFrame::HeartBeat)
            } else if cmd == 1 {
                Ok(IncomingFrame::Stop)
            } else {
                Err(FrameConversionError::InvalidFrame("Should be a data frame"))
            }
        } else {
            use core::convert::TryInto;
            let data = frame.data().unwrap();
            match cmd {
                0x0 | 0x1 => Err(FrameConversionError::InvalidFrame(
                    "Should be a remote frame",
                )),
                0x2 => {
                    check_frame_size!(2, dlc);
                    let value = i16::from_ne_bytes(data[0..2].try_into().unwrap());
                    Ok(IncomingFrame::Setpoint(value))
                }
                0x3 => {
                    check_frame_size!(1, dlc);
                    Ok(IncomingFrame::Invert(data[0] > 0))
                }
                0x4 => {
                    check_frame_size!(1, dlc);
                    Ok(IncomingFrame::SetCurrentLimit(data[0]))
                }
                _ => Err(FrameConversionError::InvalidCommand { cmd }),
            }
        }
    }
}

pub trait IntoWithId<T> {
    fn into_with_id(self, id: bxcan::Id) -> T;
}

#[derive(Format)]
pub enum OutgoingFrame {
    Update { current_now: f32, duty_now: i16 },
    Overcurrent,
    Error(u8),
}

impl IntoWithId<Frame> for OutgoingFrame {
    fn into_with_id(self, id: bxcan::Id) -> Frame {
        let mut new_id = match id {
            bxcan::Id::Standard(raw) => raw.as_raw(),
            bxcan::Id::Extended(raw) => raw.standard_id().as_raw(),
        };
        let mut bytes = heapless::Vec::<u8, heapless::consts::U8>::new();
        match self {
            OutgoingFrame::Update {
                current_now,
                duty_now,
            } => {
                new_id |= 0x1 << 8;
                bytes.extend_from_slice(duty_now.as_ne_bytes()).unwrap();
                bytes.extend_from_slice(current_now.as_ne_bytes()).unwrap();
            }
            OutgoingFrame::Overcurrent => {
                new_id |= 0x2 << 8;
            }
            OutgoingFrame::Error(code) => {
                new_id |= 0x0 << 8;
                bytes.push(code).unwrap();
            }
        };

        Frame::new_data(
            bxcan::Id::Standard(bxcan::StandardId::new(new_id).unwrap()),
            bxcan::Data::new(&bytes[..]).unwrap(),
        )
    }
}

#[derive(Debug, Format)]
pub struct PriorityFrame(pub Frame);

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
