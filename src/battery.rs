use shrewnit::{Amperes, Celsius, Current, Scalar, Temperature, Voltage, Volts};
use vexide_devices::battery;

/// Returns the electric current of the robot's battery.
///
/// Maximum current draw on the V5 battery is 20 Amps.
///
/// # Examples
///
/// ```
/// use vexide::prelude::*;
/// use vexide::devices::battery;
///
/// let current = battery::current();
///
/// println!("Drawing {} amps", current);
/// ```
pub fn current<S: Scalar>() -> Current<S> {
    Amperes * S::from_f64(battery::current()).unwrap()
}

/// Returns the internal temperature of the robot's battery.
///
/// # Examples
///
/// ```
/// use vexide::prelude::*;
/// use vexide::devices::battery;
///
/// let temp = battery::temperature();
/// println!("Battery temperature: {}°C", temp);
///
/// // Check if battery is too hot
/// if temp > 45 {
///     println!("Warning: Battery temperature critical!");
/// }
/// ```
pub fn temperature<S: Scalar>() -> Temperature<S> {
    Celsius * S::from_u64(battery::temperature()).unwrap()
}

/// Returns the robot's battery voltage.
///
/// Nominal battery voltage on the V5 brain is 12.8V.
///
/// # Examples
///
/// ```
/// use vexide::prelude::*;
/// use vexide::devices::battery;
///
/// let voltage = battery::voltage();
/// println!("Battery voltage: {} V", voltage);
/// ```
pub fn voltage<S: Scalar>() -> Voltage<S> {
    Volts * S::from_f64(battery::voltage()).unwrap()
}

#[doc(inline)]
pub use vexide_devices::battery::capacity;