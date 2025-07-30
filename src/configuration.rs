pub(crate) enum FaultBits{
}
impl FaultBits {
    pub const CJ_RANGE:u8 = 1 << 7;
    pub const TC_RANGE:u8 = 1 << 6;
    pub const CJ_HIGH:u8 = 1 << 5;
    pub const CJ_LOW:u8 = 1 << 4;
    pub const TC_HIGH:u8 = 1 << 3;
    pub const TC_LOW:u8 = 1 << 2;
    pub const OVUV:u8 = 1 << 1;
    pub const OPEN:u8 = 1 << 0;
}

/// Errors reported by the device.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceErrors {
    /// The Cold-Junction temperature is outside of the normal operating range.
    pub cold_junction_out_of_range: bool,
    /// The Thermocouple Hot Junction temperature is outside of the normal operating range.
    /// Note: The TC Range bit should be ignored in voltage mode.
    pub thermocouple_out_of_range: bool,
    /// The Cold-Junction temperature is higher than the cold-junction 
    /// temperature high threshold. The FAULT output is asserted unless masked.
    pub cold_junction_high: bool,
    /// The Cold-Junction temperature is lower than the cold-junction 
    /// temperature low threshold. The FAULT output is asserted unless masked.
    pub cold_junction_low: bool,
    /// The Thermocouple Temperature is higher than the thermocouple 
    /// temperature high threshold. The FAULT output is asserted unless masked.
    pub thermocouple_high: bool,
    /// The Thermocouple Temperature is lower than the thermocouple 
    /// temperature low threshold. The FAULT output is asserted unless masked.
    pub thermocouple_low: bool,
    /// The input voltage is negative or greater than VDD. 
    /// The FAULT output is asserted unless masked.
    /// Note: The presence of the OVUV fault will suspend conversions 
    /// and the ability of the MAX31856 to detect other faults 
    /// (or clear faults when in comparator mode) until the fault is no longer present. 
    pub overvoltage_undervoltage: bool,
    /// An open circuit such as broken thermocouple wires has been detected. 
    /// The FAULT output is asserted unless masked.
    pub open_circuit: bool,
}


/// Conversion mode
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CMode {
    /// 0 = Normally Off mode (default)
    NormallyOff = 0,
    /// 1 = Automatic Conversion mode. Conversions occur continuously every 100ms (nominal).
    AutomaticConversion = 1,
}

/// One-Shot Modes
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OneShot {
    /// 0 = No conversions requested (default)
    NoConversion = 0,
    /// 1 = This causes a single cold-junction and thermocouple conversion
    /// to take place when Conversion Mode bit =0 (normally off mode).
    // The conversion is triggered when CS goes high after writing a 1 to
    // this bit. Note that if a multi-byte write is performed, the conversion
    // is triggered when CS goes high at the end of the transaction.
    // A single conversion requires approximately 143ms in 60Hz filter mode
    // or 169ms in 50Hz filter mode to complete. This bit self clears to 0.
    OneShotConversion = 1,
}

/// Open circuit detection modes
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OCFaultModes {
    /// 00 Disabled
    Disabled = 0,
    /// 01 Enabled  RS < 5kΩ
    Enabled5k = 1,
    /// 10 Enabled 40kΩ > RS > 5kΩ; Time constant < 2ms
    Enabled40k = 2,
    /// 11 Enabled 40kΩ > RS > 5kΩ; Time constant > 2ms
    Enabled40k2ms = 3,
}

/// Fault modes
// 0 = Comparator Mode. The FAULT output and respective fault bit reflects
// the state of any nonmasked faults by asserting when the fault condition is true,
// and deasserting when the fault condition is no longer true.
// There is a 2°C hysteresis when in comparator mode for threshold fault conditions.
// 1 = Interrupt Mode. The FAULT output and respective fault bit asserts
// when a non-masked fault condition is true and remain asserted
// until a 1 is written to the Fault Status Clear bit. This deasserts FAULT
// and respective fault bit until a new fault is detected
// (note that this may occur immediately if the fault condition is still in place).
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FaultModes {
    /// Comparator Mode
    Comparator = 0,
    /// Interrupt Mode
    Interrupt = 1,
}

/// Noise rejection filter Modes
// Note: Change the notch frequency only while in the “Normally Off” mode
// – not in the Automatic conversion mode.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NoiseRejectionMode {
    /// Noise rejection for 60Hz
    Reject60Hz = 0,
    /// Noise rejection for 50Hz
    Reject50Hz = 1,
}

pub(crate) struct C0Mask {}

impl C0Mask {
    pub const CMODE_SHIFT: u8 = 7;
    pub const ONE_SHOT_SHIFT: u8 = 6;
    pub const OCFAULTPTR_SHIFT: u8 = 4;
    pub const CJ_SHIFT: u8 = 3;
    pub const FAULT_SHIFT: u8 = 2;
    pub const FAULTCLR_SHIFT: u8 = 1;
    pub const NOISEFLTR_SHIFT: u8 = 0;
}

pub(crate) struct C1Mask {}
impl C1Mask {
    pub const AVGSELECT_SHIFT: u8 = 4;
    pub const TYPE_SHIFT: u8 = 0;
}

/// Thermocouple Voltage Conversion Averaging Mode
// Adding samples increases the conversion time and reduces noise.
// Typical conversion times:
// 1-shot or first conversion in Auto mode:
// = tCONV + (samples -1) x 33.33mS (60Hz rejection)
// = tCONV + (samples -1) x 40mS (50Hz rejection)
// 2 thru n conversions in Auto mode
// = tCONV + (samples -1) x 16.67mS (60Hz rejection)
// = tCONV + (samples -1) x 20mS (50Hz rejection)
// The Averaging Mode settings SHOULD NOT be changed while
// conversions are taking place.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AveragingMode {
    /// One sample for averaging - Default
    OneSample = 0,
    /// Two samples for averaging
    TwoSamples = 1,
    /// Four samples for averaging
    FourSamples = 2,
    /// Eight samples for averaging
    EightSamples = 3,
    /// Sixteen samples for averaging
    SixteenSamples = 4,
}

/// Thermoucouple types
// 10xx = Voltage Mode, Gain = 8. Code = 8 x 1.6 x 217 x VIN
// 11xx = Voltage Mode, Gain = 32. Code = 32 x 1.6 x 217 x VIN
// Where Code is 19 bit signed number from TC registers
// and VIN is thermocouple input voltage
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThermocoupleType {
    /// B type thermocouple
    BType = 0,
    /// E type thermocouple
    EType = 1,
    /// J type thermocouple
    JType = 2,
    /// K type thermocouple
    KType = 3,
    /// N type thermocouple
    NType = 4,
    /// R type thermocouple
    RType = 5,
    /// S type thermocouple
    SType = 6,
    /// T type thermocouple
    TType = 7,
    /// Voltage Mode, Gain = 8. Code = 8 x 1.6 x 217 x VIN
    VModeGain8Type = 8,
    /// Voltage Mode, Gain = 32. Code = 32 x 1.6 x 217 x VIN
    VModeGain32Type = 9,
}

/// Options that control configuration of Max31856.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Max31856Options {
    /// Option for conversion mode
    pub conversion_mode: CMode,
    /// Option for one shot conversion
    pub one_shot_conversion: OneShot,
    /// Option for open circuit fault detection
    pub open_circuit_fault_det: OCFaultModes,

    /// Enable or disable cold junction sensor
    pub cold_junction_sensor_disable: bool,

    /// Choose fault mode
    pub fault_mode: FaultModes,

    /// Change noise rejection frequency
    pub noise_rejection_frequency: NoiseRejectionMode,

    /// Choose samples per averaging
    pub average_samples: AveragingMode,

    /// Thermocouple type selection
    pub type_selection: ThermocoupleType,
}

impl Max31856Options {
    /// Create a new set of options that can be used to derive c0 and c1 reg values
    pub fn new() -> Max31856Options {
        Max31856Options {
            conversion_mode: CMode::NormallyOff,
            one_shot_conversion: OneShot::NoConversion,
            open_circuit_fault_det: OCFaultModes::Disabled,
            cold_junction_sensor_disable: false,
            fault_mode: FaultModes::Comparator,
            noise_rejection_frequency: NoiseRejectionMode::Reject60Hz,
            average_samples: AveragingMode::OneSample,
            type_selection: ThermocoupleType::KType,
        }
    }

    /// Set conversion mode
    pub fn conversion_mode(&mut self, mode: CMode) -> &mut Self {
        self.conversion_mode = mode;
        self
    }

    /// Set one shot conversion mode
    pub fn one_shot_conversion(&mut self, mode: OneShot) -> &mut Self {
        self.one_shot_conversion = mode;
        self
    }

    /// Set open circuit fault detection mode
    pub fn open_circuit_fault_det(&mut self, mode: OCFaultModes) -> &mut Self {
        self.open_circuit_fault_det = mode;
        self
    }

    /// Set conversion mode
    pub fn cold_junction_sensor_disable(&mut self, mode: bool) -> &mut Self {
        self.cold_junction_sensor_disable = mode;
        self
    }

    /// Set fault mode
    pub fn fault_mode(&mut self, mode: FaultModes) -> &mut Self {
        self.fault_mode = mode;
        self
    }

    /// Set noise rejection frequency
    pub fn noise_rejection_frequency(&mut self, mode: NoiseRejectionMode) -> &mut Self {
        self.noise_rejection_frequency = mode;
        self
    }

    /// Set sampling count for average
    pub fn average_samples(&mut self, mode: AveragingMode) -> &mut Self {
        self.average_samples = mode;
        self
    }

    /// Set thermocouple type
    pub fn type_selection(&mut self, mode: ThermocoupleType) -> &mut Self {
        self.type_selection = mode;
        self
    }

    pub(crate) fn extract_c0(&self) -> u8 {
        let cmode = (self.conversion_mode as u8) << C0Mask::CMODE_SHIFT;
        let one_shot = (self.one_shot_conversion as u8) << C0Mask::ONE_SHOT_SHIFT;
        let ocfault_mode = (self.open_circuit_fault_det as u8) << C0Mask::OCFAULTPTR_SHIFT;
        let cj_mode = (self.cold_junction_sensor_disable as u8) << C0Mask::CJ_SHIFT;
        let fault_mode = (self.fault_mode as u8) << C0Mask::FAULT_SHIFT;
        let noise_rejection_mode =
            (self.noise_rejection_frequency as u8) << C0Mask::NOISEFLTR_SHIFT;
        cmode | one_shot | ocfault_mode | cj_mode | fault_mode | noise_rejection_mode
    }
    // Extracts register values of C0 and C1 from options.
    pub(crate) fn extract_c1(&self) -> u8 {
        let averaging_mode = (self.average_samples as u8) << C1Mask::AVGSELECT_SHIFT;
        let tc_type = (self.type_selection as u8) << C1Mask::TYPE_SHIFT;

        averaging_mode | tc_type
    }
}

impl Default for Max31856Options {
    fn default() -> Self {
        Max31856Options::new()
    }
}
