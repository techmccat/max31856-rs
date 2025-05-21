# Rust driver for MAX31856

Uses [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) 1.0.0 traits (SpiDevice) and patterns from Eldruin's [`driver-examples`](https://github.com/eldruin/driver-examples)

Communication with MAX31856 only works with Spi Mode 1 or 3.

Features:
- Modify default configuration. See: `config()`
- Read/write configuration. See: `send_config()`
- Read Linearized thermocouple temperature in Celcius. See: `temperature()`
- Read cold junction temperature. See: `ref_junction()`
- Read Fault status. See: `fault_status()`

Features in the next few versions:
- Interrupts with FAULT pin
- External temperature sensor for cold junction conversion
- Read/write fault mask registers.
- Read/write cold junction fault mask registers.
- Read/write Linearized temperature fault registers.
- Read/write cold junction temperature offset registers. 

## Usage example
```rust
use max31856;

fn example<S, FP>(spi_dev: S) -> Result<(), max31856::Error>
where
    S: embedded_hal::spi::SpiDevice,
    FP: embedded_hal::digital::InputPin,
{
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
    Ok(())
}
```
## Support

For questions, issues, feature requests like compatibility with similar devices
and other changes, please file an
[issue in the github project](https://github.com/idheepan/max31856-rs/issues).

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
