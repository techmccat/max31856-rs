//! Rust driver for MAX31856
//!
//! Uses [`embedded-hal`] 1.0.0 traits (SpiDevice) and patterns from Eldruin's [`driver-examples`]
//!
//! [`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
//! [`driver-examples`]: https://github.com/eldruin/driver-examples
//! 
//! Communication with MAX31856 only works with Spi Mode 1 or 3.
//!
//! Features:
//! - Modify default configuration. See: [`config()`]
//! - Read/write configuration. See: [`send_config()`]
//! - Read Linearized thermocouple temperature in Celcius. See: [`temperature()`]
//! - Read cold junction temperature. See: `ref_junction_temp()`
//! - Read Fault status. See: [`fault_status()`]
//!
//! [`config()`]: struct.Max31856.html#method.config
//! [`send_config()`]: struct.Max31856.html#method.send_config
//! [`temperature()`]: struct.Max31856.html#method.temperature
//! [`fault_status()`]: struct.Max31856.html#method.fault_status
//!
//! Features in the next few versions:
//! - Interrupts with FAULT pin
//! - External temperature sensor for cold junction conversion
//! - Read/write fault mask registers.
//! - Read/write cold junction fault mask registers.
//! - Read/write Linearized temperature fault registers.
//! - Read/write cold junction temperature offset registers. 
//! 
//! ## Usage example
//! ```
//! use max31856;
//! 
//! fn example<S, FP>(spi_dev: S) -> Result<(), max31856::Error>
//! where
//!     S: embedded_hal::spi::SpiDevice,
//! {
//!     let mut sensor = max31856::Max31856::new(spi_dev);
//!     // A default configuration is set on creation. It can be edited as follows
//!     sensor.config().average_samples(max31856::AveragingMode::FourSamples);
//!     let _ = sensor.send_config();
//!     println!("Temperature: {}", sensor.temperature().unwrap());
//!     sensor.config().conversion_mode(max31856::CMode::AutomaticConversion);
//!     let _ = sensor.send_config();
//!     println!("Temperature: {}", sensor.temperature().unwrap());
//!     // Faults can be assessed via 
//!     println!("Status: {:?}", sensor.fault_status()); 
//!     Ok(())
//! }
//! ```
//! 

#![deny(unsafe_code, warnings, missing_docs)]
#![no_std]
#![allow(dead_code)]

extern crate embedded_hal as hal;
use hal::spi::{Mode, MODE_3};
use configuration::FaultBits;

mod configuration;
pub use configuration::{CMode, OneShot, OCFaultModes, FaultModes, DeviceErrors, 
    NoiseRejectionMode, AveragingMode, ThermocoupleType, Max31856Options};
mod registers;
use registers::Registers;

/// Errors in this crate
#[derive(Debug)]
pub enum Error {
    /// SPI communication error
    Spi,
    /// Pin setting error
    Pin,
    /// Invalid argument provided
    InvalidArgument,
    /// Errors from the device. 
    /// Can be more than one. If there is undervoltage or overvoltage, 
    /// other errors are not detected. Fix that first. Use DeviceError
    Device(DeviceErrors),
}

/// SPI mode (CPOL = 1, CPHA = 1)
pub const MODE: Mode = MODE_3; // See Table 5. Serial Interface Function


/// Max31856 Precision Thermocouple to Digital Converter with Linearization
#[derive(Debug, Default)]
pub struct Max31856<SPI> {
    spi: SPI,
    config: Max31856Options,
}

impl<SPI> Max31856<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    /// Create a new instance of Max31856
    pub fn new(spi: SPI) -> Self {
        Max31856 {
            spi,
            config: Max31856Options::default(),
        }
    }

    /// Parse options and write to C0 and C1 registers. 
    pub fn send_config(&mut self) -> Result<(), Error> {
        let buf = [
            Registers::CR0.write_address,
            self.config.extract_c0(),
            self.config.extract_c1(),
        ];
        self.spi.write(&buf)
            .map_err(|_| Error::Spi)
    }

    fn send_c0(&mut self) -> Result<(), Error> {
        self.spi.write(&[Registers::CR0.write_address, self.config.extract_c0()])
        .map_err(|_| Error::Spi)
    }

    fn send_c1(&mut self) -> Result<(), Error> {
        self.spi.write(&[Registers::CR1.write_address, self.config.extract_c1()])
        .map_err(|_| Error::Spi)
    }

    /// Get a reference of stored configuration. This can be then used to modify certain
    /// values. send_config() can then be used to write it to the sensor. 
    pub fn config(&mut self) -> &mut Max31856Options{
        &mut self.config
    }
    /// Sets the enabled (unmasked) fault conditions
    /// on which the FAULT pin is pulled low
    ///
    /// Fault will never be pulled low for cold junction or thermocouple out of range,
    /// trying to enable them will result in an InvalidArgument error
    pub fn set_fault_mask(&mut self, enabled: DeviceErrors) -> Result<(), Error> {
        if enabled.cold_junction_out_of_range || enabled.thermocouple_out_of_range {
            return Err(Error::InvalidArgument)
        }
        let mut mask = Registers::MASK.factory_default;
        if enabled.open_circuit { mask &= !FaultBits::OPEN }
        if enabled.overvoltage_undervoltage { mask &= !FaultBits::OVUV }
        if enabled.thermocouple_low { mask &= !FaultBits::TC_LOW }
        if enabled.thermocouple_high { mask &= !FaultBits::TC_HIGH }
        if enabled.cold_junction_low { mask &= !FaultBits::CJ_LOW }
        if enabled.cold_junction_high { mask &= !FaultBits::CJ_HIGH }

        self.spi.write(&[Registers::MASK.write_address, mask])
            .map_err(|_| Error::Spi)
    }

    /// Set cold junction temperature thresholds
    pub fn set_cj_threshold(&mut self, low: Option<i8>, high: Option<i8>) -> Result<(), Error> {
        let high = high.map(|s| s as u8)
            .unwrap_or(Registers::CJHF.factory_default);
        let low = low.map(|s| s as u8)
            .unwrap_or(Registers::CJLF.factory_default);
        let buf = [Registers::CJHF.write_address, high, low];
        self.spi.write(&buf).map_err(|_| Error::Spi)
    }

    /// Set thermocouple temperature thresholds
    pub fn set_thermocouple_threshold(
        &mut self,
        low: Option<f32>,
        high: Option<f32>
    ) -> Result<(), Error> {
        let f_to_reg = |f: f32| {
            let i = (f * 16.0) as i16;
            let low = (i & 0xFF) as u8;
            let high = (i >> 8) as u8;
            (high, low)
        };
        let high = high.map(f_to_reg)
            .unwrap_or((
                Registers::LTHFTH.factory_default,
                Registers::LTHFTL.factory_default
            ));
        let low = low.map(f_to_reg)
            .unwrap_or((
                Registers::LTLFTH.factory_default,
                Registers::LTLFTL.factory_default
            ));
        let buf = [Registers::CJHF.write_address, high.0, high.1, low.0, low.1];
        self.spi.write(&buf).map_err(|_| Error::Spi)
    }

    //TODO: method for cold junction temperature offset

    /// Triggers a temperature conversion
    ///
    /// In single conversion mode this takes at most 155ms at 60Hz, 185ms as 50Hz  
    /// No-op if the IC is in automatic conversion mode
    pub fn trigger_conversion(&mut self) -> Result<(), Error> {
        let cmode = self.config.conversion_mode;
        match cmode {
            CMode::NormallyOff => {
                self.config.one_shot_conversion = OneShot::OneShotConversion;
                self.send_c0() //One shot only changes c0. This part is executed often
            }
            _ => Ok(()),
        } 
    }
    /// Retrieves probe and cold junction temperature with a single measurement
    pub fn probe_and_cj_temperature(&mut self) -> Result<(f32, f32), Error> {
        //If conversion mode is normally off, a one-time conversion should be done.
        //The one shot conversion takes about 150ms and then the bit is reset.
        //On automatic conversion mode, the temperature can requested without 1-shot trigger
        self.trigger_conversion()?;

        // reading two extra bytes is a rounding error when compared to the conversion time
        // addr + t_ref + t_probe + fault
        let mut buf= [0u8; 1 + 2 + 3 + 1];
        buf[0] = Registers::CJTH.read_address;
        self.spi.transfer_in_place(&mut buf).map_err(|_| Error::Spi)?;

        fault_to_result(buf[6])?;

        // q7.8 format, 1 LSB = 2^-8
        let t_ref = i16::from_be_bytes([buf[1], buf[2]]) as f32 / 256.0; // 2^8

        let t_probe = {
            // The three bits are rearranged to derive the temperature
            let sign = if buf[3] & 0x80 == 0x80 {-1.0} else {1.0};
            let mut value = ((buf[3] & 0x7F) as i32) << 24;
            value += (buf[4] as i32) << 16;
            value += (buf[5] as i32) << 8;
            sign * (value as f32)/1048576.0 // 2^10
        };

        Ok((t_probe, t_ref))
    }

    /// Get the measured value of cold-junction temperature 
    /// plus the value in the Cold-Junction Offset register
    pub fn ref_junction_temp(&mut self) -> Result<f32, Error> {
        self.probe_and_cj_temperature().map(|t| t.1)
    }

    /// Get the linearized and cold-junction-compensated thermocouple
    /// temperature value.
    pub fn temperature(&mut self) -> Result<f32, Error>{
        self.probe_and_cj_temperature().map(|t| t.0)
    }

    /// Check if any of the faults are triggered
    pub fn fault_status(&mut self) -> Result<(), Error>{
        let mut buffer = [0u8; 2]; // One byte value from Fault status register
        buffer[0] = Registers::SR.read_address;
        self.spi.transfer_in_place(&mut buffer).map_err(|_| Error::Spi)?;

        fault_to_result(buffer[1])
    }
}

fn fault_to_result(error_id: u8) -> Result<(), Error> {
    if error_id == 0 { return Ok(()) }
    let mut has_error = false;
    let mut errors = DeviceErrors::default();

    //If overvoltage or undervoltage, all other errors might not be set
    if(error_id & FaultBits::OVUV) !=0 {
        errors.overvoltage_undervoltage = true;
        return Err(Error::Device(errors))
    }
    if(error_id & FaultBits::CJ_HIGH) !=0 {
        errors.cold_junction_high = true;
        has_error = true;
    }
    if(error_id & FaultBits::CJ_LOW) !=0 {
        errors.cold_junction_low = true;
        has_error = true;
    }
    if(error_id & FaultBits::CJ_RANGE) !=0 {
        errors.cold_junction_out_of_range = true;
        has_error = true;
    }
    if(error_id & FaultBits::OPEN) !=0 {
        errors.open_circuit = true;
        has_error = true;
    }
    if(error_id & FaultBits::TC_HIGH) !=0 {
        errors.thermocouple_high = true;
        has_error = true;
    }
    if(error_id & FaultBits::TC_LOW) !=0 {
        errors.thermocouple_low = true;
        has_error = true;
    }
    if(error_id & FaultBits::TC_RANGE) !=0 {
        errors.thermocouple_out_of_range = true;
        has_error = true;
    }
    if has_error {            
        Err(Error::Device(errors))
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod internal {
    use super::*;
    #[test]
    fn can_extract_max31856_c0_c1() {
        let mut options = Max31856Options::new();
        options.average_samples(AveragingMode::SixteenSamples)
            .fault_mode(FaultModes::Interrupt)
            .noise_rejection_frequency(NoiseRejectionMode::Reject50Hz)
            .conversion_mode(CMode::AutomaticConversion);
        let c0_c1 = (options.extract_c0(), options.extract_c1());
        assert_eq!(c0_c1, (0b1000_0101, 0b0100_0011));
    }
}
