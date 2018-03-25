#![no_std]
mod registers;
mod band;
mod util;


#[macro_use]
extern crate register;

#[macro_use]
extern crate arrayref;

#[macro_use]
extern crate matches;

extern crate byteorder;
extern crate embedded_hal as hal;
extern crate crc16;
extern crate fpa;
extern crate cast;

use register::*;
use band::*;
use util::*;

use core::marker::PhantomData;
use core::any::TypeId;

//use byteorder::{ByteOrder, BE};

use registers::Register;

use cast::{usize, u16, u32};
use hal::blocking::delay::DelayMs;
use hal::blocking;
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};


// TODO: Use u16 Extension instead of casting
pub struct Rfm12b<SPI, NCS, INT, RESET, BAND> {
    ncs: NCS,
    int: INT,
    reset: RESET,
    spi: SPI,

    _ism_band: PhantomData<BAND>,

    state: State,

    // Max payload length is 128 bytes
    // 7 bytes overhead
    buffer: [u8; 135],

    packet_length: u8,

    pattern: u8,

}

impl<E, SPI, NCS, INT, RESET, BAND> Rfm12b<SPI, NCS, INT, RESET, BAND>
    where
        SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
        NCS: OutputPin,
        INT: IntPin + InputPin,
        RESET: ResetPin,
        BAND: Rfm12bBand + 'static,
{
    pub fn new<D>(
        spi: SPI,
        ncs: NCS,
        int: INT,
        reset: RESET,
        delay: &mut D,
        mut rx_buf_sz: u16,
        src: [u8; 6],
        pattern: u8
    ) -> Result<Self, Error<E>> where
        D: DelayMs<u8>,
        RESET: ResetPin,
        INT: IntPin,
    {
        let mut rfm12b = Rfm12b {
            spi: spi,
            ncs: ncs,
            int: int,
            reset: reset,
            state: State::Init,
            _ism_band: PhantomData,
            buffer: [0; 135],
            packet_length: 0,
            pattern: pattern,
        };

        // NOTE: We leave Frequency at default (434.15 Mhz)
        // We also leave Baudrate at default(9600 bps)
        // These values should be set accordingly later in the configuration process

        // Read the status (this is required to get radio outp of reset state)
        rfm12b.read_register(Register::StatusRead)?;

        delay.delay_ms(100);

        let baseband_id = if TypeId::of::<BAND>() == TypeId::of::<Rfm12bMhz433>() {
            band::Rfm12bMhz433::baseband_id()
        } else {
            return Err(Error::ConfigError);
        };

        rfm12b.write_register(Register::ConfigSetting,
                              registers::ConfigSetting::<Write>::default()
                                  .enable_data_register(1)
                                  .enable_fifo_mode(1)
                                  .freq_band(baseband_id) // TODO: Make this general
                                  .crystal_load_capacitance(7) // 12pF
                                  .get() as u16
        )?;
        rfm12b.write_register(Register::PowerManagement,
                              registers::PowerManagement::<Write>::default()
                                  .enable_receiver_chain(1)
                                  .enable_synthesizer(1) // enable synthesizer (faster switch to TX)
                                  .disable_clock_output(1)
                                  .get() as u16
        )?;


        rfm12b.write_register(Register::RxControl,
                              registers::RxControl::<Write>::default()
                              .p16_func(1) // Set P16 to output VDI signal
                              .vdi_signal_response_time(3) // Slow (Best quality)W
                              .rssi_thresh(2) // -91dB
                              .get() as u16
        )?;
        rfm12b.write_register(Register::DataFilter,
                              registers::DataFilter::<Write>::default()
                                  .auto_clock_recovery_lock(1)
                                  //.filter_type(0) // Internal Digital
                                  .dqd_threshold(4) // 200kHz TODO: Make this settable
                                  .get() as u16

        )?;
        rfm12b.write_register(Register::FifoAndResetMode,
                              registers::FifoAndResetMode::<Write>::default()
                                  .enable_fifo_fill(1) // Enable fifo fill
                                  .sensitive_reset(1) // Disable sensitive reset
                                  .get() as u16
        )?;
        rfm12b.write_register(Register::SynchPattern,
                              registers::SynchPattern::<Write>::default()
                                  .synchron_pattern(pattern)
                                  .get() as u16
        )?;
        rfm12b.write_register(Register::Afc,
                              registers::Afc::<Write>::default()
                                  .enable_calculation(0) // deactivate (unusable)
                                  .get() as u16
        )?;
        rfm12b.write_register(Register::TxConfig,
                              registers::TxConfig::<Write>::default()
                                  .output_power(2) // -10 dB
                                  .get() as u16
        )?;
        rfm12b.write_register(Register::PllSetting,
                              registers::PllSetting::<Write>::default()
                                  .low_power_xtal(0) // disable
                                  .get() as u16
        )?;

        rfm12b.state = State::Idle;
        Ok(rfm12b)
    }

    /// Destroys the driver and returns all the hardware resources that were owned by it
    pub fn free(self) -> (SPI, NCS, INT, RESET) {
        (self.spi, self.ncs, self.int, self.reset)
    }

    // TODO: Handle interups
    pub fn interrupt_handle() -> () {}


    pub fn send(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        assert!(bytes.len() < 128);


        if self.state == State::Idle {

            self.state = State::Send(3);

            // Encode the packet
            self.encode_packet(bytes);

            // Start sending
            self.write_register(Register::TxRegisterWrite, 0xAA)?;
            self.write_register(Register::TxRegisterWrite, 0xAA)?;

            // If no interupt pin start polling for next bytes
            if TypeId::of::<INT>() == TypeId::of::<Unconnected>() {
                for i in 3..self.packet_length {
                    self._send()?;
                }
            }


        } else {
            return Err(Error::TransmitterBusyError);
        }

        Ok(())
    }

    fn _send(&mut self) -> Result<(), Error<E>> {
        match self.state {
            State::Send(b) => {
                let byte = self.buffer[b as usize];
                self.write_register(Register::TxRegisterWrite, byte as u16)?;
            },
            _ => return Err(Error::StateError),
        }

        Ok(())
    }

    fn encode_packet(&mut self, bytes: &[u8]) {
        assert!(matches!(self.state, State::Send(_)));

        self.packet_length = bytes.len() as u8;

        // Calculate CRC
        let mut crc16 = crc16::State::<crc16::ARC>::new();
        crc16.update(self.buffer.as_ref());

        let mut buf = &mut self.buffer;

        *array_mut_ref![buf, 0, 7] = [0xAA, 0xAA, 0x2D, self.pattern, self.packet_length,
            (crc16.get() >> 8) as u8, crc16.get() as u8];

        for i in 7..self.packet_length+7 {
             buf[i as usize] = bytes[i as usize];
        }
    }

    // FIXME: totally based on assumptions
    fn decode_packet(&mut self, bytes: &mut [u8]) -> Result<u8, Error<E>> {
        assert!(matches!(self.state, State::Receive(_)));

        // TODO: Check, that preamble and synchron pattern are not received
        // TODO: Check that no former

        let mut buf = &mut self.buffer;
        self.packet_length = buf[0];

        for i in 3..self.packet_length+3 {
            bytes[i as usize] = buf[i as usize];
        }

        // Calculate CRC
        let mut crc16 = crc16::State::<crc16::ARC>::new();
        crc16.update(buf);

        if crc16.get() == ( (buf[1] as u16) << 8 | buf [2] as u16) {
            Ok(self.packet_length)
        } else {
            Err(Error::Crc16Error)
        }
    }


    fn read_register(&mut self, register: Register) -> Result<u16, E> {
        self.ncs.set_low();
        let mut buffer: [u8; 2] = [register.addr(), 0];
        self.spi.transfer(&mut buffer)?;
        self.ncs.set_high();

        Ok((buffer[0] as u16) << 8 | buffer[1] as u16)
    }

    fn write_register(&mut self, register: Register, value: u16) -> Result<(), E> {
        self.ncs.set_low();
        let buffer = [register.addr() | (value >> 8) as u8, value as u8];
        self.spi.write(&buffer)?;
        self.ncs.set_high();

        Ok(())
    }

    fn modify_register<F>(&mut self, register: Register, f: F) -> Result<(), E>
        where
            F: FnOnce(u16) -> u16,
    {
        let r = self.read_register(register)?;
        self.write_register(register, f(r))
    }

    fn bit_field_set(&mut self, register: Register, mask: u16) -> Result<(), E> {
        self.ncs.set_low();
        self.spi.write(&[register.addr() | (mask >> 8) as u8, mask as u8])?;
        self.ncs.set_high();

        Ok(())
    }

}

// TODO: Move this into util
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

    #[test]
    fn array_ref_test() {
        let mut buf: &mut [u8; 4] = &mut [0; 4];
        *array_mut_ref![buf, 0, 2] = [0xAA, 0xAA];
        assert_eq!(buf, &[0xAA, 0xAA, 0x00, 0x00]);
    }
}

