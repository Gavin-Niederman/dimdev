use core::time::Duration;

use shrewnit::{Angle, AngularVelocity, Dimension, Radians, RadiansPerSecond};
use vexide_devices::{
    position::Position, smart::{motor::Direction, rotation, SmartDevice, SmartDeviceType, SmartPort}, PortError
};

/// A rotation sensor plugged into a Smart Port.
#[derive(Debug, PartialEq)]
pub struct RotationSensor {
    inner: rotation::RotationSensor,
}

impl RotationSensor {
    /// The minimum data rate that you can set a rotation sensor to.
    pub const MIN_DATA_INTERVAL: Duration = rotation::RotationSensor::MIN_DATA_INTERVAL;

    /// The amount of unique sensor readings per one revolution of the sensor.
    pub const TICKS_PER_REVOLUTION: u32 = rotation::RotationSensor::TICKS_PER_REVOLUTION;

    /// Creates a new rotation sensor on the given port.
    ///
    /// Whether or not the sensor should be reversed on creation can be specified.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    /// }
    /// ```
    #[must_use]
    pub fn new(port: SmartPort, direction: Direction) -> Self {
        Self {
            inner: rotation::RotationSensor::new(port, direction),
        }
    }

    /// Reset's the sensor's position reading to zero.
    ///
    /// # Errors
    ///
    /// An error is returned if a rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     println!("Before reset: {:?}", sensor.position());
    ///
    ///     _ = sensor.reset_position();
    ///
    ///     println!("After reset: {:?}", sensor.position());
    /// }
    /// ```
    pub fn reset_position(&mut self) -> Result<(), PortError> {
        self.inner.reset_position()
    }

    /// Sets the sensor's position reading.
    ///
    /// # Errors
    ///
    /// An error is returned if a rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     // Set position to 15 degrees.
    ///     _ = sensor.set_position(Position::from_degrees(15.0));
    /// }
    /// ```
    pub fn set_position(&mut self, position: Angle) -> Result<(), PortError> {
        self.inner
            .set_position(Position::from_radians(position.to::<Radians>()))
    }

    /// Sets the sensor to operate in a given [`Direction`].
    ///
    /// This determines which way the sensor considers to be “forwards”. You can use the marking on the top of the
    /// motor as a reference:
    ///
    /// - When [`Direction::Forward`] is specified, positive velocity/voltage values will cause the motor to rotate
    ///   **with the arrow on the top**. Position will increase as the motor rotates **with the arrow**.
    /// - When [`Direction::Reverse`] is specified, positive velocity/voltage values will cause the motor to rotate
    ///   **against the arrow on the top**. Position will increase as the motor rotates **against the arrow**.
    ///
    /// # Errors
    ///
    /// - An error is returned if an rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the sensor's direction to [`Direction::Reverse`].
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     // Reverse the sensor
    ///     _ = sensor.set_direction(Direction::Reverse);
    /// }
    /// ```
    ///
    /// Reverse the sensor's direction (set to opposite of the previous direction):
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     // Reverse the sensor
    ///     _ = sensor.set_direction(!sensor.direction());
    /// }
    /// ```
    pub fn set_direction(&mut self, new_direction: Direction) -> Result<(), PortError> {
        self.inner.set_direction(new_direction)
    }

    /// Sets the internal computation speed of the rotation sensor.
    ///
    /// This method does NOT change the rate at which user code can read data off the sensor, as the brain will only talk to
    /// the device every 10mS regardless of how fast data is being sent or computed. See [`RotationSensor::UPDATE_INTERVAL`].
    ///
    /// This duration should be above [`Self::MIN_DATA_INTERVAL`] (5 milliseconds).
    ///
    /// # Errors
    ///
    /// An error is returned if an rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     // Set to minimum interval.
    ///     _ = sensor.set_data_interval(RotationSensor::MIN_DATA_INTERVAL);
    /// }
    /// ```
    pub fn set_computation_interval(&mut self, interval: Duration) -> Result<(), PortError> {
        self.inner.set_computation_interval(interval)
    }

    /// Returns the [`Direction`] of this sensor.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     println!(
    ///         "Sensor's direction is {}",
    ///         match sensor.direction() {
    ///             Direction::Forward => "forward",
    ///             Direction::Reverse => "reverse",
    ///         }
    ///     );
    /// }
    /// ```
    #[must_use]
    pub const fn direction(&self) -> Direction {
        self.inner.direction()
    }

    /// Returns the angular rotation of the sensor based on direction.
    ///
    /// # Errors
    ///
    /// An error is returned if an rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     if let Ok(position) = sensor.position() {
    ///         println!("Position in degrees: {}°", position.as_degrees());
    ///         println!("Position in radians: {}°", position.as_radians());
    ///         println!("Position in raw ticks (centidegrees): {}°", position.as_ticks(RotationSensor::TICKS_PER_REVOLUTION));
    ///         println!("Number of revolutions spun: {}°", position.as_revolutions());
    ///     }
    /// }
    /// ```
    pub fn position(&self) -> Result<Angle, PortError> {
        self.inner.position().map(|pos| pos.as_radians() * Radians)
    }

    /// Returns the angle of rotation measured by the sensor.
    /// 
    /// # Errors
    ///
    /// An error is returned if an rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     if let Ok(angle) = sensor.angle() {
    ///         println!("Angle in degrees: {}°", angle.as_degrees());
    ///         println!("Angle in radians: {}°", angle.as_radians());
    ///         println!("Angle in raw ticks (centidegrees): {}°", angle.as_ticks(RotationSensor::TICKS_PER_REVOLUTION));
    ///     }
    /// }
    /// ```
    pub fn angle(&self) -> Result<Angle, PortError> {
        self.inner.angle().map(|pos| pos.as_radians() * Radians)
    }

    /// Returns the sensor's current velocity in degrees per second.
    ///
    /// # Errors
    ///
    /// An error is returned if an rotation sensor is not currently connected to the Smart Port.
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     if let Some(velocity) = sensor.velocity() {
    ///         println!(
    ///             "Velocity in RPM {}",
    ///             velocity / 6.0, // 1rpm = 6dps
    ///         );
    ///     }
    /// }
    /// ```
    pub fn velocity(&self) -> Result<AngularVelocity, PortError> {
        self.inner.velocity().map(|vel| vel * RadiansPerSecond)
    }

    /// Returns the sensor's internal status code.
    ///
    /// # Errors
    ///
    /// An error is returned if an rotation sensor is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = RotationSensor::new(peripherals.port_1, Direction::Forward);
    ///
    ///     if let Ok(status) = sensor.status() {
    ///         println!("Status: {:b}", status);
    ///     }
    /// }
    /// ```
    pub fn status(&self) -> Result<u32, PortError> {
        self.inner.status()
    }
}

impl SmartDevice for RotationSensor {
    fn port_number(&self) -> u8 {
        self.inner.port_number()
    }

    fn device_type(&self) -> SmartDeviceType {
        SmartDeviceType::Rotation
    }
}
impl From<RotationSensor> for SmartPort {
    fn from(device: RotationSensor) -> Self {
        SmartPort::from(device.inner)
    }
}