use defmt::Format;

#[derive(Format, PartialEq, Eq)]
#[repr(u8)]
pub enum ErrorCode {
    None = 0,
    MotorDriverFault = 1,
    CanError = 2,
    Other = 3,
}

impl From<ErrorCode> for u8 {
    fn from(code: ErrorCode) -> u8 {
        code as u8
    }
}
