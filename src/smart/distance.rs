use shrewnit::{Length, LinearVelocity, MetersPerSecond, Millimeters, Scalar};
use vexide_devices::smart::{distance, SmartDevice, SmartDeviceType, SmartPort};

pub use distance::DistanceError;

/// Readings from a physical object detected by a Distance Sensor.
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub struct DistanceObject<S: Scalar> {
    /// The distance of the object from the sensor.
    pub distance: Length<S>,

    /// A guess at the object's "relative size".
    ///
    /// This is a value that has a range of 0 to 400. A 18" x 30" grey card will return
    /// a value of approximately 75 in typical room lighting. If the sensor is not able to
    /// detect an object, None is returned.
    ///
    /// This sensor reading is unusual, as it is entirely unitless with the seemingly arbitrary
    /// range of 0-400 existing due to VEXCode's [`vex::sizeType`] enum having four variants. It's
    /// unknown what the sensor is *actually* measuring here either, so use this data with a grain
    /// of salt.
    ///
    /// [`vex::sizeType`]: https://api.vexcode.cloud/v5/search/sizeType/sizeType/enum
    pub relative_size: u32,

    /// Observed velocity of the object.
    pub velocity: LinearVelocity<S>,

    /// Returns the confidence in the distance measurement from 0.0 to 1.0.
    pub confidence: f64,
}
impl<S: Scalar> From<distance::DistanceObject> for DistanceObject<S> {
    fn from(object: distance::DistanceObject) -> Self {
        Self {
            distance: Millimeters * S::from_u32(object.distance).unwrap(),
            relative_size: object.relative_size,
            velocity:  MetersPerSecond * S::from_f64(object.velocity).unwrap(),
            confidence: object.confidence,
        }
    }
}

pub struct DistanceSensor {
    inner: distance::DistanceSensor,
}

impl DistanceSensor {
    /// Creates a new distance sensor from a [`SmartPort`].
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = DistanceSensor::new(peripherals.port_1);
    /// }
    /// ```
    #[must_use]
    pub fn new(port: SmartPort) -> Self {
        Self {
            inner: distance::DistanceSensor::new(port),
        }
    }

    /// Attempts to detect an object, returning `None` if no object could be found.
    ///
    /// # Errors
    ///
    /// - A [`DistanceError::Port`] error is returned if there is not a distance sensor connected to the port.
    /// - A [`DistanceError::StillInitializing`] error is returned if the distance sensor is still initializing.
    /// - A [`DistanceError::BadStatusCode`] error is returned if the distance sensor has an unknown status code.
    ///
    /// # Examples
    ///
    /// Measure object distance and velocity:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = DistanceSensor::new(peripherals.port_1);
    ///
    ///     if let Some(object) = sensor.object().unwrap_or_default() {
    ///         println!("Object of size {}mm is moving at {}m/s", object.distance, object.velocity);
    ///     }
    /// }
    /// ```
    ///
    /// Get object distance, but only with high confidence:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = DistanceSensor::new(peripherals.port_1);
    ///
    ///     let distance = sensor.object()
    ///         .unwrap_or_default()
    ///         .and_then(|object| {
    ///             if object.confidence > 0.8 {
    ///                 Some(object.distance)
    ///             } else {
    ///                 None
    ///             }
    ///         });
    /// }
    /// ```
    pub fn object<S: Scalar>(&self) -> Result<Option<DistanceObject<S>>, DistanceError> {
        self.inner.object().map(|object| object.map(Into::into))
    }

    /// Returns the internal status code of the distance sensor.
    /// The status code of the signature can tell you if the sensor is still initializing or if it is working correctly.
    /// If the distance sensor is still initializing, the status code will be 0x00.
    /// If it is done initializing and functioning correctly, the status code will be 0x82 or 0x86.
    ///
    /// # Errors
    ///
    /// - A [`DistanceError::Port`] error is returned if there is not a distance sensor connected to the port.
    ///
    /// # Examples
    ///
    /// A simple initialization state check:
    ///
    /// ```
    /// use vexide::prelude::*;
    /// use core::time::Duration;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let distance_sensor = DistanceSensor::new(peripherals.port_1);
    ///     loop {
    ///         if let Ok(0) = distance_sensor.status() {
    ///             println!("Sensor is still initializing");
    ///         } else {
    ///             println!("Sensor is ready");
    ///         }
    ///         sleep(Duration::from_millis(10)).await;
    ///     }
    /// }
    /// ```
    ///
    /// Printing the status code in binary format:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let sensor = DistanceSensor::new(peripherals.port_1);
    ///
    ///     if let Ok(status) = sensor.status() {
    ///         println!("Status: {:b}", status);
    ///     }
    /// }
    /// ```
    pub fn status(&self) -> Result<u32, DistanceError> {
        self.inner.status()
    }
}

impl SmartDevice for DistanceSensor {
    fn port_number(&self) -> u8 {
        self.inner.port_number()
    }

    fn device_type(&self) -> SmartDeviceType {
        SmartDeviceType::Distance
    }
}
impl From<DistanceSensor> for SmartPort {
    fn from(device: DistanceSensor) -> Self {
        SmartPort::from(device.inner)
    }
}