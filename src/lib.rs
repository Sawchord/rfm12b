#![no_std]
mod registers;
mod band;


#[macro_use]
extern crate register;

extern crate byteorder;
extern crate embedded_hal as hal;
extern crate crc16;
extern crate fpa;
extern crate cast;

use byteorder::{ByteOrder, BE};
use core::marker::PhantomData;

use registers::Register;
use register::*;

use band::*;

use cast::{usize, u16, u32};
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

    ism_band: PhantomData<BAND>,

    pattern: u8,

    /// Running CRC value
    crc16: crc16::State<crc16::ARC>,

    // MAC address of this node
    //mac: u16,

}

// TODO: Is there a better way to implement this for all bands, without reimplementing all the functions
// impl<E, SPI, NCS, INT, RESET, BAND>
impl<E, SPI, NCS, INT, RESET> Rfm12b<SPI, NCS, INT, RESET, Rfm12bMhz433>
    where
        SPI: blocking::spi::Transfer<u8, Error = E> + blocking::spi::Write<u8, Error = E>,
        NCS: OutputPin,
        INT: IntPin + InputPin,
        RESET: ResetPin,
        //BAND: Rfm12bBand,
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
            ism_band: PhantomData,
            pattern: pattern,
        };


        // Read the status (this is required to get radio outp of reset state)
        let status = rfm12b.read_register(Register::StatusRead)?;

        // Poll until device is started up
        //while rfm12b.int.is_high() {}
        
        delay.delay_ms(100);

        rfm12b.write_register(Register::ConfigSetting,
                              registers::ConfigSetting::<Write>::default()
                                  .enable_data_register(1)
                                  .enable_fifo_mode(1)
                                  .freq_band(band::Rfm12bMhz433::baseband_id()) // TODO: Make this general
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

        // NOTE: We leave Frequency at default (434.15)
        // We also leave Baudrate at default(9600)
        // These values should be set accordingly later in the configuration process

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

        Ok(rfm12b)
    }

    /// Destroys the driver and returns all the hardware resources that were owned by it
    pub fn free(self) -> (SPI, NCS, INT, RESET) {
        (self.spi, self.ncs, self.int, self.reset)
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
