use defmt::Format;

use core::convert::Into;

#[derive(Format, PartialEq, Eq)]
#[repr(u8)]
pub enum ErrorCode {
    None = 0,
    MotorDriverFault = 1,
    CanError = 2,
    Other = 3,
}

impl Into<u8> for ErrorCode {
    fn into(self) -> u8 {
        self as u8
    }
}
