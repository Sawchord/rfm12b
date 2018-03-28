
use core::mem;
use byteorder::{ByteOrder, BE, LE};

use hal::digital::{InputPin, OutputPin};

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum State {
    Init,
    Idle,
    Send(u8),
    PollingReceive,
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


pub struct Unconnected;
pub unsafe trait ResetPin: 'static {
    #[doc(hidden)]
    fn reset(&mut self);
}

unsafe impl ResetPin for Unconnected {
    fn reset(&mut self) {}
}

unsafe impl<OP> ResetPin for OP
    where
        OP: OutputPin + 'static,
{
    fn reset(&mut self) {
        self.set_low();
        self.set_high();
    }
}

pub unsafe trait IntPin: 'static {}


impl InputPin for Unconnected {
    fn is_high(&self) -> bool {
        return false;
    }

    fn is_low(&self) -> bool {
        return false;
    }
}

unsafe impl<IP> IntPin for IP
    where
        IP: InputPin + 'static,
{}