
use fpa::I16F16;
use core::result::{Result};
use cast::{u8, u16};
// The access traits of the registers

#[derive(Debug, Copy, Clone)]
pub enum Rfm12bConfigError {
    BelowMinFreq,
    AboveMaxFreq,
    ChannelNotSupportedError,
    ConversionError,
}


pub trait Rfm12bBand {
    fn BasebandId() -> u8;

    fn MinFreq() -> I16F16;
    fn MaxFreq() -> I16F16;

    fn ChannelToFreq(u16) -> Result<I16F16, Rfm12bConfigError>;
    fn FreqToRegister(freq: I16F16) -> Result<u16, Rfm12bConfigError>;

}

#[derive(Clone, Copy)]
pub struct Rfm12bMhz433;
impl Rfm12bBand for Rfm12bMhz433 {

    fn BasebandId() -> u8 {
        1
    }

    fn MinFreq() -> I16F16 {
        I16F16(433.05_f32).unwrap()
    }

    fn MaxFreq() -> I16F16 {
        I16F16(434.79_f32).unwrap()
    }

    fn FreqToRegister(freq: I16F16) -> Result<u16, Rfm12bConfigError>{

        // NOTE: These are not the technicly possible values, but rather the
        // frequencies specified for the band
        if freq < Self::MinFreq()  {
            return Err(Rfm12bConfigError::BelowMinFreq);
        }

        if freq > Self::MaxFreq() {
            return Err(Rfm12bConfigError::BelowMinFreq);
        }


        match (u16(((freq/10) - 43 * 4000))) {
            Ok(num) => Ok(num),
            Err(err) => Err(Rfm12bConfigError::ConversionError),
        }
    }

    fn ChannelToFreq(chan: u16) -> Result<I16F16, Rfm12bConfigError> {
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