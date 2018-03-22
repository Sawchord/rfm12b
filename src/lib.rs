#![no_std]
mod registers;

#[macro_use]
extern crate register;

extern crate byteorder;
extern crate embedded_hal as hal;
extern crate crc16;

// The access traits of the registers
#[derive(Clone, Copy)]
pub struct Mhz433;

#[derive(Clone, Copy)]
pub struct Mhz868;

#[derive(Clone, Copy)]
pub struct Mhz915;


use byteorder::{ByteOrder, BE};
use core::marker::PhantomData;

use registers::Register;
use register::*;

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

    // address of the next packet in buffer memory
    // NOTE this should be an `Option` but we know this is a 13-bit address so we'll use `u16::MAX`
    // as the `None` variant, to avoid an extra byte (enum tag)
    //rx_ptr: u8,


    // End of the RX buffer / Start of the TX buffer
    //tx_ptr: u8,

    /// Running CRC value
    crc16: crc16::State<crc16::ARC>,

    // MAC address of this node
    //mac: u16,

}

// TODO: Replace Register types with Into<Register> traits
impl<E, SPI, NCS, INT, RESET, BAND> Rfm12b<SPI, NCS, INT, RESET, BAND>
    where
        SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
        NCS: OutputPin,
        INT: IntPin + InputPin,
        RESET: ResetPin,
{
    pub fn new<D>(
        spi: SPI,
        ncs: NCS,
        int: INT,
        reset: RESET,
        delay: &mut D,
        mut rx_buf_sz: u16,
        src: [u8; 6],
    ) -> Result<Self, E> where
        D: DelayMs<u8>,
        RESET: ResetPin,
        INT: IntPin,
    {
        let mut rfm12b = Rfm12b {
            spi: spi,
            ncs: ncs,
            int: int,
            reset: reset,
            crc16: crc16::State::new(),
            _ism_band: PhantomData,
        };


        // Read the status (this is required to get radio outp of reset state)
        let status = rfm12b.read_register::<Register>(Register::StatusRead)?;

        // Disable clock output
        // Go into FSK mode
        // Poll until device is started up
        while rfm12b.int.is_high() {}

        //while registers::StatusRead(rfm12b.read_register::<Register>(Register::StatusRead)?).por() == 1 {}
        // Set capacitance
        // Set Band
        // Set Frequency
        // Set transmission rate
        // Other stuff

        Ok(rfm12b)
    }

    /// Destroys the driver and returns all the hardware resources that were owned by it
    pub fn free(self) -> (SPI, NCS, INT, RESET) {
        (self.spi, self.ncs, self.int, self.reset)
    }

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

    fn modify_register<R, F>(&mut self, register: Register, f: F) -> Result<(), E>
        where
            F: FnOnce(u16) -> u16,
            R: Into<Register>,
    {
        let r = self.read_register::<Register>(register)?;
        self.write_register::<Register>(register, f(r))
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
