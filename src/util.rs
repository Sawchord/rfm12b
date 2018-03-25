use core::mem;
use byteorder::{ByteOrder, BE, LE};

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum State {
    Init,
    Idle,
    Send(u8),
    Receive(u8),
}

#[derive(Clone, Debug)]
pub enum Error<E> {
    StateError,
    ConfigError,
    TransmitterBusyError,
    Crc16Error,

    // TODO: Implement correctly, to distinguish SPI, Config and Send Errors
    Spi(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Spi(e)
    }
}

pub trait U16Ext {
    fn from_bytes(low: u8, high: u8) -> Self;
    fn low(self) -> u8;
    fn high(self) -> u8;
    fn be_repr(self) -> [u8; 2];
    fn le_repr(self) -> [u8; 2];
}

impl U16Ext for u16 {
    fn from_bytes(low: u8, high: u8) -> u16 {
        ((high as u16) << 8) + low as u16
    }

    fn low(self) -> u8 {
        (self & 0xff) as u8
    }

    fn high(self) -> u8 {
        (self >> 8) as u8
    }

    fn be_repr(self) -> [u8; 2] {
        let mut bytes: [u8; 2] = unsafe { mem::uninitialized() };
        BE::write_u16(&mut bytes, self);
        bytes
    }

    fn le_repr(self) -> [u8; 2] {
        let mut bytes: [u8; 2] = unsafe { mem::uninitialized() };
        LE::write_u16(&mut bytes, self);
        bytes
    }
}
