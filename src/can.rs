use bxcan::{Data, ExtendedId, Frame, Id, StandardId};

use core::cmp::Ordering;

use core::convert::{Into, TryFrom};

use defmt::Format;

pub struct InvalidFrameError;

// TODO: implement this
// impl Format for InvalidFrameError;

#[derive(Format)]
pub enum IncomingFrame {
    Setpoint(i16),
    SetCurrentLimit(u8),
    Invert(bool),
    HeartBeat,
    Stop,
}

impl TryFrom<Frame> for IncomingFrame {
    type Error = InvalidFrameError;
    fn try_from(frame: Frame) -> Result<IncomingFrame, Self::Error> {
        unimplemented!();
    }
}

#[derive(Format)]
pub enum OutgoingFrame {
    Update,
    CurrentLimit,
}

impl Into<Frame> for OutgoingFrame {
    fn into(self) -> Frame {
        unimplemented!();
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
