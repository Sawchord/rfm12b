#![no_std]

extern crate register;
extern crate embedded_hal as hal;

// The access traits of the registers
#[derive(Clone, Copy)]
pub struct Mhz433;

#[derive(Clone, Copy)]
pub struct Mhz868;

#[derive(Clone, Copy)]
pub struct Mhz915;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
