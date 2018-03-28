#![no_std]
mod registers;
pub mod band;
pub mod util;


#[macro_use]
extern crate register;

#[macro_use]
extern crate arrayref;

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

use fpa::{I16F16, I7F1};

use registers::Register;

use cast::{usize, u16, u32};
use hal::blocking::delay::DelayUs;
use hal::blocking;
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnFirstTransition,
    polarity: Polarity::IdleLow,
};


// TODO: Use u16 Extension instead of casting
pub struct Rfm12b<SPI, NCS, INT, RESET, BAND, D> {
    ncs: NCS,
    int: INT,
    reset: RESET,
    spi: SPI,

    _ism_band: PhantomData<BAND>,

    delay: D,

    // Max payload length is 128 bytes
    // 7 bytes overhead
    buffer: [u8; 135],
    packet_length: u8,
    pattern: u8,

    freq: I16F16,
    baud: u32,

    state: State,
}

impl<E, SPI, NCS, INT, RESET, BAND, D> Rfm12b<SPI, NCS, INT, RESET, BAND, D>
    where
        SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
        NCS: OutputPin,
        INT: IntPin + InputPin,
        RESET: ResetPin,
        BAND: Rfm12bBand + 'static,
        D: DelayUs<u32>,
{
    pub fn new(
        spi: SPI,
        ncs: NCS,
        int: INT,
        reset: RESET,
        delay: D,
        pattern: u8
    ) -> Result<Self, Error<E>> where
        RESET: ResetPin,
        INT: IntPin,
    {
        let mut rfm12b = Rfm12b {
            spi: spi,
            ncs: ncs,
            int: int,
            reset: reset,
            _ism_band: PhantomData,
            delay: delay,
            buffer: [0; 135],
            packet_length: 0,
            pattern: pattern,
            freq: I16F16(434.15_f32).unwrap(),
            baud: 9600,
            state: State::Init,
        };

        // NOTE: We leave Frequency at default (434.15 Mhz)
        // We also leave Baudrate at default(9600 bps)
        // These values should be set accordingly later in the configuration process

        // Read the status (this is required to get radio outp of reset state)
        rfm12b.read_register(Register::StatusRead)?;

        rfm12b.delay.delay_us(100000);

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

    // TODO: Handle interrupts
    // TODO: Implement callback style packet delivery mechanism
    pub fn interrupt_handle() -> () {}


    pub fn send(&mut self, bytes: &[u8]) -> Result<(), Error<E>> {
        assert!(bytes.len() < 128);


        if self.state != State::Idle {
            return Err(Error::TransmitterBusyError);
        }

        self.state = State::Send(3);

        // Encode the packet
        self.encode_packet(bytes);

        // Fill send fifo with first bytes
        self.write_register(Register::TxRegisterWrite, 0xAA)?;
        self.write_register(Register::TxRegisterWrite, 0xAA)?;

        // Go into send mode
        self.modify_register(Register::PowerManagement,
                             |w| registers::PowerManagement(w as u8)
                                 .modify()
                                 .enable_sender_chain(1)
                                 .get() as u16);

        // If no interrupt pin start polling for next bytes
        if TypeId::of::<INT>() == TypeId::of::<Unconnected>() {
            for i in 3..self.packet_length {

                // To reduce traffic, we delay before polling
                self.delay.delay_us(1_000_000/self.baud);

                while registers::StatusRead(self.read_register(Register::StatusRead)?).tx_ready() != 0 {}
                self._send()?;
            }
        }
        // If interrupts are enabled, the interrup_handle function will manage sending the
        // rest of the data
        Ok(())
    }

    pub fn polling_receive(&mut self, bytes: &mut [u8]) -> Result<u8, Error<E>> {

        if self.state != State::Idle {
            return  Err(Error::StateError);
        }
        self.state = State::PollingReceive;

        while self.state != State::Idle {

            if registers::StatusRead(
                self.read_register(Register::StatusRead)?).rx_ready() == 1 {
                self._receive()?;
            }

            // Slow down polling to reduce bus delay
            self.delay.delay_us(500_000/self.baud);
        }
        Ok(self.decode_packet(bytes)?)
    }

    fn _send(&mut self) -> Result<(), Error<E>> {
        match self.state {
            State::Send(b) => {
                let byte = self.buffer[b as usize];
                self.write_register(Register::TxRegisterWrite, byte as u16)?;
                self.state = State::Send(b+1);
            },
            _ => return Err(Error::StateError),
        }

        Ok(())
    }

    fn _receive(&mut self) -> Result<(), Error<E>> {
        match self.state {
            State::Receive(b) => {
                let byte = self.read_register(Register::RxRegisterRead)? as u8;

                self.buffer[b as usize] = byte;

                if b == 0 {
                    self.packet_length = byte;
                } else if b >= self.packet_length {
                    self.state = State::Idle;
                }

            },
            _ => return Err(Error::StateError),
        };

        Ok(())
    }

    fn encode_packet(&mut self, bytes: &[u8]) {
        //assert!(matches!(self.state, State::Send(_)));
        // TODO: Find replacement for this assert

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
        //assert!(matches!(self.state, State::Receive(_)));
        // TODO: Find replacement for this assert

        // TODO: Check, that preamble and synchron pattern are not received
        // TODO: Need to clear buffer at any point

        let mut buf = &mut self.buffer;
        //self.packet_length = buf[0];

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

