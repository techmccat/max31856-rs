use max31856;

use embedded_hal;
use embedded_hal_bus::spi::ExclusiveDevice;

// fake stuff for example
struct FakeSpiBus();

impl embedded_hal::spi::ErrorType for FakeSpiBus {
    type Error = core::convert::Infallible;
}

impl embedded_hal::spi::SpiBus<u8> for FakeSpiBus {
    fn read(&mut self, _: &mut [u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn write(&mut self, _: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Self::Error> {
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

struct FakeCs();

impl embedded_hal::digital::ErrorType for FakeCs {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for FakeCs {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct FakeDelayer();

impl embedded_hal::delay::DelayNs for FakeDelayer {
    fn delay_ns(&mut self, ns: u32) {
        std::thread::sleep(std::time::Duration::from_nanos(u64::from(ns)));
    }
}

fn main () {
    // BEGIN fake stuff that has to be replaced with real peripherals
    let spi_bus = FakeSpiBus();
    let delay = FakeDelayer();
    let spi_dev = ExclusiveDevice::new(spi_bus, FakeCs(), delay).unwrap();
    // END fake stuff that has to be replaced with real peripherals

    let mut sensor = max31856::Max31856::new(spi_dev);
    // A default configuration is set on creation. It can be edited as follows
    sensor.config().average_samples(max31856::AveragingMode::FourSamples);
    let _ = sensor.send_config();
    println!("Temperature: {}", sensor.temperature().unwrap());
    sensor.config().conversion_mode(max31856::CMode::AutomaticConversion);
    let _ = sensor.send_config();
    println!("Temperature: {}", sensor.temperature().unwrap());
    // Faults can be assessed via 
    println!("Status: {:?}", sensor.fault_status()); 
}
