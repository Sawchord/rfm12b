#![no_std]
mod registers;

#[macro_use]
extern crate register;

extern crate byteorder;
extern crate embedded_hal as hal;
extern crate crc16;

use registers::Register;

// The access traits of the registers
#[derive(Clone, Copy)]
pub struct Mhz433;

#[derive(Clone, Copy)]
pub struct Mhz868;

#[derive(Clone, Copy)]
pub struct Mhz915;


use byteorder::{ByteOrder, BE};
use core::marker::PhantomData;

//use register::Register;

//use cast::{usize, u16, u32W};
use hal::blocking::delay::DelayMs;
use hal::blocking;
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};


pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};

pub struct Rfm12b<SPI, NCS, INT, RESET, BAND> {
    int: INT,
    ncs: NCS,
    reset: RESET,
    spi: SPI,

    _ism_band: PhantomData<BAND>,

    /// address of the next packet in buffer memory
    // NOTE this should be an `Option` but we know this is a 13-bit address so we'll use `u16::MAX`
    // as the `None` variant, to avoid an extra byte (enum tag)
    rx_ptr: u8,


    /// End of the RX buffer / Start of the TX buffer
    tx_ptr: u8,

    /// Running CRC value
    crc16: u16,

    /// MAC address of this node
    mac: u16,

}

// TODO: Replace Register types with Into<Register> traits
impl<E, SPI, NCS, INT, RESET, BAND> Rfm12b<SPI, NCS, INT, RESET, BAND>
    where
        SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
        NCS: OutputPin,
        INT: IntPin,
        RESET: ResetPin,
{

    fn read_register<R>(&mut self, register: Register) -> Result<u16, E>
        where
            R: Into<Register>,
    {
        self.ncs.set_low();
        let mut buffer: [u8; 2] = [register.addr(), 0];
        self.spi.transfer(&mut buffer)?;
        self.ncs.set_high();

        Ok((buffer[0] as u16) << 8 | buffer[1] as u16)
    }

    fn write_register<R>(&mut self, register: Register, value: u16) -> Result<(), E>
        where
            R: Into<Register>,
    {
        self.ncs.set_low();
        let buffer = [register.addr() | (value >> 8) as u8, value as u8];
        self.spi.write(&buffer)?;
        self.ncs.set_high();

        Ok(())
    }

}

pub struct Unconnected;

// FIXME this should be a closed set trait
/// [Implementation detail] Reset pin
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

unsafe impl IntPin for Unconnected {}

unsafe impl<IP> IntPin for IP
    where
        IP: InputPin + 'static,
{
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
