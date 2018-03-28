
use fpa::I16F16;
use core::result::{Result};
//use core::convert::{From};
use cast::{u8, u16};
// The access traits of the registers

#[derive(Debug, Copy, Clone)]
pub enum Rfm12bConfigError {
    BelowMinFreqError,
    AboveMaxFreqError,
    ChannelNotSupportedError,
    ConversionError,
    BitRateNotSupportedError,
    BandNotSupportedError,
}

// TODO: Baseband to register
// TODO: OutputPower to register

pub trait Rfm12bBand {
    fn baseband_id() -> u8;

    fn min_freq() -> I16F16;
    fn max_freq() -> I16F16;

    fn channel_to_freq(chan: u16) -> Result<I16F16, Rfm12bConfigError>;
    fn freq_to_register(freq: I16F16) -> Result<u16, Rfm12bConfigError>;

    fn baudrate_to_register(bps: u32) -> Result<(bool, u8), Rfm12bConfigError> {
        if bps < 600 || bps > 115200 {
            return Err(Rfm12bConfigError::BitRateNotSupportedError)
        }

        let cs0_reg = (10_000_000 / (29 * bps)) as u32;
        let cs1_reg = (10_000_000 / (29 * bps * 8)) as u32;

        if cs1_reg > 127 {
            Ok((true, cs1_reg as u8))
        } else {
            Ok((false, cs0_reg as u8))
        }
    }

}

#[derive(Clone, Copy)]
pub struct Rfm12bMhz433;
impl Rfm12bBand for Rfm12bMhz433 {

    fn baseband_id() -> u8 {
        1
    }

    fn min_freq() -> I16F16 {
        I16F16(433.05_f32).unwrap()
    }

    fn max_freq() -> I16F16 {
        I16F16(434.79_f32).unwrap()
    }

    fn freq_to_register(freq: I16F16) -> Result<u16, Rfm12bConfigError>{

        // NOTE: These are not the technicly possible values, but rather the
        // frequencies specified for the band
        if freq < Self::min_freq()  {
            return Err(Rfm12bConfigError::BelowMinFreqError);
        }

        if freq > Self::max_freq() {
            return Err(Rfm12bConfigError::BelowMinFreqError);
        }


        match u16((freq/10) - 43 * 4000) {
            Ok(num) => Ok(num),
            Err(err) => Err(Rfm12bConfigError::ConversionError),
        }
    }

    fn channel_to_freq(chan: u16) -> Result<I16F16, Rfm12bConfigError> {
        if chan < 1 || chan > 69 {
            return  Err(Rfm12bConfigError::ChannelNotSupportedError)
        }
        // FIXME: Should not use unwrap on chan (why does it not compile without unwrap?)
        Ok(I16F16(433.075_f32).unwrap() + I16F16(0.025_f32).unwrap() * I16F16(chan).unwrap())
    }

}


// TODO: Somebody find out these values
#[derive(Clone, Copy)]
pub struct Rfm12bMhz868;

#[derive(Clone, Copy)]
pub struct Rfm12bMhz915;