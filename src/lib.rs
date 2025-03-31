//! A platform agnostic driver to interface with the L3GD20 (gyroscope)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/1.0
//!
//! # Examples
//!
//! You should find at least one example in the [f3] crate.
//!
//! [f3]: https://docs.rs/f3/0.6

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

use embedded_hal::spi::Mode;
use embedded_hal::spi::SpiDevice;

/// Minimal time in nanoseconds between chip select assertion and clock edge.
pub const MINIMUM_CS_SETUP_TIME_NS: u32 = 5;

/// Expected WHO_AM_I register value for the L3GD20 sensor.
pub const WHO_AM_I_L3GD20: u8 = 0xD4;
/// Expected WHO_AM_I register value for the L3GD20H sensor.
pub const WHO_AM_I_L3GD20H: u8 = 0xD7;

/// SPI mode
pub const MODE: Mode = embedded_hal::spi::MODE_3;

/// L3GD20 driver
pub struct L3gd20<Spi> {
    spi: Spi,
}

impl<Spi: SpiDevice> L3gd20<Spi>
{
    /// Creates a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: Spi) -> Result<Self, Spi::Error> {
        let mut l3gd20 = L3gd20 { spi };

        // power up and enable all the axes
        l3gd20.write_register(Register::CTRL_REG1, 0b0000_1111)?;

        Ok(l3gd20)
    }

    /// Temperature measurement + gyroscope measurements
    pub fn all(&mut self) -> Result<Measurements, Spi::Error> {
        let mut bytes = [0u8; 9];
        self.read_many(Register::OUT_TEMP, &mut bytes)?;

        Ok(Measurements {
            gyro: I16x3 {
                x: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
                y: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
                z: (bytes[7] as u16 + ((bytes[8] as u16) << 8)) as i16,
            },
            temp_raw: bytes[1] as i8,
        })
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, Spi::Error> {
        let mut bytes = [0u8; 7];
        self.read_many(Register::OUT_X_L, &mut bytes)?;

        Ok(I16x3 {
            x: (bytes[1] as u16 + ((bytes[2] as u16) << 8)) as i16,
            y: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
            z: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
        })
    }

    /// Raw temperature sensor measurement
    pub fn temp_raw(&mut self) -> Result<i8, Spi::Error> {
        Ok(self.read_register(Register::OUT_TEMP)? as i8)
    }

    /// Actual temperature derived by subtracting the raw measurement to the baseline value of 25 C
    pub fn temp_celcius(&mut self) -> Result<i16, Spi::Error> {
        Ok(25 - self.temp_raw()? as i16)
    }

    /// Reads the WHO_AM_I register; should return `0xD4`
    pub fn who_am_i(&mut self) -> Result<u8, Spi::Error> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Read `STATUS_REG` of sensor
    pub fn status(&mut self) -> Result<Status, Spi::Error> {
        let sts = self.read_register(Register::STATUS_REG)?;
        Ok(Status::from_u8(sts))
    }

    /// Get the current Output Data Rate
    pub fn odr(&mut self) -> Result<Odr, Spi::Error> {
        // Read control register
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        Ok(Odr::from_u8(reg1))
    }

    /// Set the Output Data Rate
    pub fn set_odr(&mut self, odr: Odr) -> Result<&mut Self, Spi::Error> {
        self.change_config(Register::CTRL_REG1, odr)
    }

    /// Get current Bandwidth
    pub fn bandwidth(&mut self) -> Result<Bandwidth, Spi::Error> {
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        Ok(Bandwidth::from_u8(reg1))
    }

    /// Set low-pass cut-off frequency (i.e. bandwidth)
    ///
    /// See `Bandwidth` for further explanation
    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> Result<&mut Self, Spi::Error> {
        self.change_config(Register::CTRL_REG1, bw)
    }

    /// Get the current Full Scale Selection
    ///
    /// This is the sensitivity of the sensor, see `Scale` for more information
    pub fn scale(&mut self) -> Result<Scale, Spi::Error> {
        let scl = self.read_register(Register::CTRL_REG4)?;
        Ok(Scale::from_u8(scl))
    }

    /// Set the Full Scale Selection
    ///
    /// This sets the sensitivity of the sensor, see `Scale` for more
    /// information
    pub fn set_scale(&mut self, scale: Scale) -> Result<&mut Self, Spi::Error> {
        self.change_config(Register::CTRL_REG4, scale)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, Spi::Error> {
        let mut buffer = [reg.addr() | SINGLE | READ, 0];
        self.spi.transfer_in_place(&mut buffer)?;

        Ok(buffer[1])
    }

    /// Read multiple bytes starting from the `start_reg` register.
    /// This function will attempt to fill the provided buffer.
    fn read_many(&mut self, start_reg: Register, buffer: &mut [u8]) -> Result<(), Spi::Error> {
        buffer[0] = start_reg.addr() | MULTI | READ;
        self.spi.transfer_in_place(buffer)?;

        Ok(())
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), Spi::Error> {
        let buffer = [reg.addr() | SINGLE | WRITE, byte];
        self.spi.write(&buffer)?;

        Ok(())
    }

    /// Change configuration in register
    ///
    /// Helper function to update a particular part of a register without
    /// affecting other parts of the register that might contain desired
    /// configuration. This allows the `L3gd20` struct to be used like
    /// a builder interface when configuring specific parameters.
    fn change_config<B: BitValue>(&mut self, reg: Register, bits: B) -> Result<&mut Self, Spi::Error> {
        // Create bit mask from width and shift of value
        let mask = B::mask() << B::shift();
        // Extract the value as u8
        let bits = (bits.value() << B::shift()) & mask;
        // Read current value of register
        let current = self.read_register(reg)?;
        // Use supplied mask so we don't affect more than necessary
        let masked = current & !mask;
        // Use `or` to apply the new value without affecting other parts
        let new_reg = masked | bits;
        self.write_register(reg, new_reg)?;
        Ok(self)
    }
}

/// Trait to represent a value that can be sent to sensor
trait BitValue {
    /// The width of the bitfield in bits
    fn width() -> u8;
    /// The bit 'mask' of the value
    fn mask() -> u8 {
        (1 << Self::width()) - 1
    }
    /// The number of bits to shift the mask by
    fn shift() -> u8;
    /// Convert the type to a byte value to be sent to sensor
    ///
    /// # Note
    /// This value should not be bit shifted.
    fn value(&self) -> u8;
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Register {
    WHO_AM_I = 0x0F,
    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,
    REFERENCE = 0x25,
    OUT_TEMP = 0x26,
    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL_REG = 0x2E,
    FIFO_SRC_REG = 0x2F,
    INT1_CFG = 0x30,
    INT1_SRC = 0x31,
    INT1_TSH_XH = 0x32,
    INT1_TSH_XL = 0x33,
    INT1_TSH_YH = 0x34,
    INT1_TSH_YL = 0x35,
    INT1_TSH_ZH = 0x36,
    INT1_TSH_ZL = 0x37,
    INT1_DURATION = 0x38,
}

/// Output Data Rate
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Odr {
    /// 95 Hz data rate
    Hz95 = 0x00,
    /// 190 Hz data rate
    Hz190 = 0x01,
    /// 380 Hz data rate
    Hz380 = 0x02,
    /// 760 Hz data rate
    Hz760 = 0x03,
}

impl BitValue for Odr {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        6
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Odr {
    fn from_u8(from: u8) -> Self {
        // Extract ODR value, converting to enum (ROI: 0b1100_0000)
        match (from >> Odr::shift()) & Odr::mask() {
            x if x == Odr::Hz95 as u8 => Odr::Hz95,
            x if x == Odr::Hz190 as u8 => Odr::Hz190,
            x if x == Odr::Hz380 as u8 => Odr::Hz380,
            x if x == Odr::Hz760 as u8 => Odr::Hz760,
            _ => unreachable!(),
        }
    }
}

/// Full scale selection
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Scale {
    /// 250 Degrees Per Second
    Dps250 = 0x00,
    /// 500 Degrees Per Second
    Dps500 = 0x01,
    /// 2000 Degrees Per Second
    Dps2000 = 0x03,
}

impl BitValue for Scale {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        4
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Scale {
    fn from_u8(from: u8) -> Self {
        // Extract scale value from register, ensure that we mask with
        // `0b0000_0011` to extract `FS1-FS2` part of register
        match (from >> Scale::shift()) & Scale::mask() {
            x if x == Scale::Dps250 as u8 => Scale::Dps250,
            x if x == Scale::Dps500 as u8 => Scale::Dps500,
            x if x == Scale::Dps2000 as u8 => Scale::Dps2000,
            // Special case for Dps2000
            0x02 => Scale::Dps2000,
            _ => unreachable!(),
        }
    }
}

/// Bandwidth of sensor
///
/// The bandwidth of the sensor is equal to the cut-off for the low-pass
/// filter. The cut-off depends on the `Odr` of the sensor, for specific
/// information consult the data sheet.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bandwidth {
    /// Lowest possible cut-off for any `Odr` configuration
    Low = 0x00,
    /// Medium cut-off, can be the same as `High` for some `Odr` configurations
    Medium = 0x01,
    /// High cut-off
    High = 0x02,
    /// Maximum cut-off for any `Odr` configuration
    Maximum = 0x03,
}

impl BitValue for Bandwidth {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        4
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Bandwidth {
    fn from_u8(from: u8) -> Self {
        // Shift and mask bandwidth of register, (ROI: 0b0011_0000)
        match (from >> Bandwidth::shift()) & Bandwidth::mask() {
            x if x == Bandwidth::Low as u8 => Bandwidth::Low,
            x if x == Bandwidth::Medium as u8 => Bandwidth::Medium,
            x if x == Bandwidth::High as u8 => Bandwidth::High,
            x if x == Bandwidth::Maximum as u8 => Bandwidth::Maximum,
            _ => unreachable!(),
        }
    }
}

const READ: u8 = 1 << 7;
const WRITE: u8 = 0 << 7;
const MULTI: u8 = 1 << 6;
const SINGLE: u8 = 0 << 6;

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Scale {
    /// Convert a measurement to degrees
    pub fn degrees(&self, val: i16) -> f32 {
        match *self {
            Scale::Dps250 => val as f32 * 0.00875,
            Scale::Dps500 => val as f32 * 0.0175,
            Scale::Dps2000 => val as f32 * 0.07,
        }
    }

    /// Convert a measurement to radians
    pub fn radians(&self, val: i16) -> f32 {
        // TODO: Use `to_radians` or other built in method
        // NOTE: `to_radians` is only exported in `std` (07.02.18)
        self.degrees(val) * (core::f32::consts::PI / 180.0)
    }
}

/// XYZ triple
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Several measurements
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurements {
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Raw temperature sensor measurement
    pub temp_raw: i8,
}

impl Measurements {
    /// Convert the raw temperature value to degrees celcius
    pub fn temp_celcius(&self) -> i16 {
        25 - self.temp_raw as i16
    }
}

/// Sensor status
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Overrun (data has overwritten previously unread data)
    /// has occurred on at least one axis
    pub overrun: bool,
    /// Overrun occurred on Z-axis
    pub z_overrun: bool,
    /// Overrun occurred on Y-axis
    pub y_overrun: bool,
    /// Overrun occurred on X-axis
    pub x_overrun: bool,
    /// New data is available for either X, Y, Z - axis
    pub new_data: bool,
    /// New data is available on Z-axis
    pub z_new: bool,
    /// New data is available on Y-axis
    pub y_new: bool,
    /// New data is available on X-axis
    pub x_new: bool,
}

impl Status {
    fn from_u8(from: u8) -> Self {
        Status {
            overrun: (from & (1 << 7)) != 0,
            z_overrun: (from & (1 << 6)) != 0,
            y_overrun: (from & (1 << 5)) != 0,
            x_overrun: (from & (1 << 4)) != 0,
            new_data: (from & (1 << 3)) != 0,
            z_new: (from & (1 << 2)) != 0,
            y_new: (from & (1 << 1)) != 0,
            x_new: (from & 1) != 0,
        }
    }
}
