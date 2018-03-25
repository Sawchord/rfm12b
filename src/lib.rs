#![no_std]
mod registers;
mod band;
mod util;


#[macro_use]
extern crate register;

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
    transmission_state: Transmission,


    buffer: [u8; 128],

    packet_length: u8,

    pattern: u8,

    /// Running CRC value
    crc16: crc16::State<crc16::ARC>,
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
            transmission_state: Transmission::None,
            crc16: crc16::State::new(),
            _ism_band: PhantomData,
            buffer: [0; 128],
            packet_length: 0,
            pattern: pattern,
        };

        // NOTE: We leave Frequency at default (434.15)
        // We also leave Baudrate at default(9600)
        // These values should be set accordingly later in the configuration process

        // Read the status (this is required to get radio outp of reset state)
        let status = rfm12b.read_register(Register::StatusRead)?;

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

    pub fn send(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        assert!(bytes.len() < 128);


        if self.state == State::Idle {
            self.state = State::Send;

            // Copy the buffer into local memory, the
            self.buffer.clone_from_slice(bytes);

            // Initialize CRC
            self.crc16 = crc16::State::new();
            self.crc16.update(self.buffer.as_ref());

            // Start sending
            self.write_register(Register::TxRegisterWrite, 0xAA)?;
            self.write_register(Register::TxRegisterWrite, 0xAA)?;
            self.transmission_state = Transmission::Preamble(0);

            // If no input start polling for next bytes
            if TypeId::of::<INT>() == TypeId::of::<Unconnected>() {
                while self.transmission_state != Transmission::None {
                    self._send(bytes);
                }
            }


        } else {
            return Err(Error::TransmitterBusyError);
        }

        Ok(())
    }

    fn _send(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        match self.transmission_state {
            Transmission::Preamble(0) => {
                self.write_register(Register::TxRegisterWrite, 0x2D)?;
                self.transmission_state = Transmission::Preamble(1);
            },
            Transmission::Preamble(1) => {
                let pattern = self.pattern;
                self.write_register(Register::TxRegisterWrite, pattern as u16)?;
                self.transmission_state = Transmission::Payload(0);
                },
            Transmission::Payload(b) => {
                let byte = self.buffer[b as usize];
                self.write_register(Register::TxRegisterWrite, byte as u16)?;

                if b < self.packet_length {
                    self.transmission_state = Transmission::Payload(b+1);
                } else {
                    self.transmission_state = Transmission::Crc16(0);
                }
            },
            Transmission::Crc16(0) => {
                let crc = self.crc16.get();
                self.write_register(Register::TxRegisterWrite, crc >> 8)?;
                self.transmission_state = Transmission::Crc16(1);
            }
            Transmission::Crc16(1) => {
                let crc = self.crc16.get();
                self.write_register(Register::TxRegisterWrite, crc)?;
                self.transmission_state = Transmission::None;
            }
            _ => return Err(Error::StateError),
        };
        
        Ok(())
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
}

