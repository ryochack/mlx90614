//! A platform agnostic driver to interface the MLX90614 (Infra Red Thermometer)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//!
//! # Examples
//!
//! You'll find an example for the Raspeberry Pi in the `examples` directory. You should find an
//! example for ARM Cortex-M microcontrollers on the [`blue-pill`] repository. If that branch is
//! gone, check the master branch.
//!
//! [`blue-pill`]: https://github.com/japaric/blue-pill/tree/singletons/examples
//!
//! # References
//!
//! [MLX90614 data sheet]: https://www.melexis.com/-/media/files/documents/datasheets/mlx90614-datasheet-melexis.pdf

#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::i2c;

#[derive(Debug)]
pub enum Error<E> {
    /// EEEPROM is busy
    EepromBusy,
    /// EEEPROM is fatal error
    EepromDead,
    /// Packet Error Code
    PacketErrorCode,
    /// I2C bus error
    I2c(E),
}

pub const DEFAULT_SLAVE_ADDR: u8 = 0x5A;

struct SlaveAddress(u8);

impl SlaveAddress {
    fn address(&self) -> u8 {
        self.0
    }
    fn read_address(&self) -> u8 {
        (self.0 << 1) | 1
    }
    fn write_address(&self) -> u8 {
        (self.0 << 1) | 0
    }
}

pub enum EepromAddr {
    ToMax = 0x00,
    ToMin = 0x01,
    PwmCtrl = 0x02,
    TaRange = 0x03,
    Emissivity = 0x04,
    ConfigRegister1 = 0x05,
    SMBussAddress = 0x0E,
    ID0 = 0x1C,
    ID1 = 0x1D,
    ID2 = 0x1E,
    ID3 = 0x1F,
}

pub enum RamAddr {
    RawDataIrChannel1 = 0x04,
    RawDataIrChannel2 = 0x05,
    Ta = 0x06,
    Tobj1 = 0x07,
    Tobj2 = 0x08,
}

#[derive(Copy, Clone)]
pub enum Register {
    // RAM Access
    RawDataIrChannel1 = 0x04,
    RawDataIrChannel2 = 0x05,
    Ta = 0x06,
    Tobj1 = 0x07,
    Tobj2 = 0x08,
    // EEPROM Access
    ToMax = 0x20,
    ToMin = 0x21,
    PwmCtrl = 0x22,
    TaRange = 0x23,
    Emissivity = 0x24,
    ConfigRegister1 = 0x25,
    SMBussAddress = 0x2E,
    ID0 = 0x3C,
    ID1 = 0x3D,
    ID2 = 0x3E,
    ID3 = 0x3F,
    // Read Flags
    ReadFlags = 0xF0,
    // Enter Sleep Mode
    Sleep = 0xFF,
}

impl Register {
    fn as_byte(&self) -> u8 {
        *self as u8
    }
}

struct ReadFlags {
    eeprom_is_busy: bool,
    eeprom_is_dead: bool,
    has_initilized: bool,
}

fn read_flags(flags: u8) -> ReadFlags {
    const EEBUSY_BIT: u8 = 0x80;
    const EE_DEAD_BIT: u8 = 0x20;
    const INIT_BIT: u8 = 0x10;

    ReadFlags {
        eeprom_is_busy: flags & EEBUSY_BIT == EEBUSY_BIT,
        eeprom_is_dead: flags & EE_DEAD_BIT == EE_DEAD_BIT,
        has_initilized: flags & INIT_BIT == INIT_BIT,
    }
}

fn crc8(a: u8, b: u8) -> u8 {
    let mut data: u8 = a ^ b;
    for _ in 0..8 {
        if data & 0x80 != 0 {
            data <<= 1;
            data ^= 0x07;
        } else {
            data <<= 1;
        }
    }
    data
}

pub struct Mlx90614<I2C> {
    smbus: I2C,
    slave: SlaveAddress,
}

impl<E, I2C> Mlx90614<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::WriteRead<Error = E>
{
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let mlx90614 = Mlx90614 {
            smbus: i2c,
            slave: SlaveAddress(DEFAULT_SLAVE_ADDR),
        };

        Ok(mlx90614)
    }

    pub fn write(&mut self, reg: Register, data: &u16) -> Result<(), E> {
        let mut buf: [u8; 5] = [
            self.slave.write_address(),
            reg.as_byte(),
            (data & 0x00FFu16) as u8,
            ((data >> 8) & 0x00FFu16) as u8,
            0,
        ];
        let pec = buf[..buf.len()-1].iter().fold(0, |crc, i| crc8(crc, *i));
        buf[buf.len()-1] = pec;

        self.smbus.write(self.slave.address(), &buf[1..])
    } 

    pub fn read(&mut self, reg: Register) -> Result<u16, Error<E>> {
        let mut buf: [u8; 6] = [
            self.slave.write_address(),
            reg.as_byte(),
            self.slave.read_address(),
            0, 0, 0,
        ];
        
        self.smbus.write_read(self.slave.address(), &[reg.as_byte()], &mut buf[3..]).map_err(Error::I2c)?;

        let pec = buf[..buf.len()-1].iter().fold(0, |crc, i| crc8(crc, *i));
        if pec != buf[buf.len()-1] {
            return Err(Error::PacketErrorCode);
        } 
        Ok((buf[3] as u16) | ((buf[4] as u16) << 8))
    } 
}

#[cfg(test)]
mod tests {
    fn it_works() {
    }
}
