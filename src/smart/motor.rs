use core::time::Duration;

use num_traits::ToPrimitive;
use shrewnit::{
    Amperes, Angle, AngularVelocity, Celsius, Current, Dimension, Power, Radians, RotationsPerMinute, Scalar, Temperature, Voltage, Volts, Watts
};
use vexide_devices::{
    position::Position,
    smart::{
        motor::{self, BrakeMode, Direction, Gearset, MotorError, MotorFaults, MotorStatus, MotorType}, SmartPort
    },
};

#[cfg(feature = "dangerous_motor_tuning")]
use vexide_devices::smart::motor::MotorTuningConstants;

/// A possible target action for a [`Motor`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotorControl<S: Scalar> {
    /// The motor brakes using a specified [`BrakeMode`].
    Brake(BrakeMode),

    /// The motor outputs a raw voltage.
    ///
    /// # Fields
    ///
    /// - `0`: The desired output voltage of the motor
    Voltage(Voltage<S>),

    /// The motor attempts to hold a velocity using its internal PID control.
    ///
    /// # Fields
    ///
    /// - `0`: The desired speed of the motor during the movement operation
    Velocity(AngularVelocity<S>),

    /// The motor attempts to reach a position using its internal PID control.
    ///
    /// # Fields
    ///
    /// - `0`: The desired position of the motor after the movement operation
    /// - `1`: The desired speed of the motor during the movement operation
    Position(Angle<S>, AngularVelocity<S>),
}
impl<S: Scalar + ToPrimitive> From<MotorControl<S>> for motor::MotorControl {
    fn from(value: MotorControl<S>) -> Self {
        match value {
            MotorControl::Brake(mode) => motor::MotorControl::Brake(mode),
            MotorControl::Voltage(voltage) => {
                motor::MotorControl::Voltage(voltage.to::<Volts>().to_f64().unwrap())
            }
            MotorControl::Velocity(velocity) => {
                motor::MotorControl::Velocity(velocity.to::<RotationsPerMinute>().to_i32().unwrap())
            }
            MotorControl::Position(position, velocity) => motor::MotorControl::Position(
                Position::from_radians(position.to::<Radians>().to_f64().unwrap()),
                velocity.to::<RotationsPerMinute>().to_i32().unwrap(),
            ),
        }
    }
}
impl<S: Scalar> From<motor::MotorControl> for MotorControl<S> {
    fn from(value: motor::MotorControl) -> Self {
        match value {
            motor::MotorControl::Brake(brake_mode) => MotorControl::Brake(brake_mode),
            motor::MotorControl::Voltage(voltage) => {
                MotorControl::Voltage(Volts * S::from_f64(voltage).unwrap())
            }
            motor::MotorControl::Velocity(velocity) => {
                MotorControl::Velocity(RotationsPerMinute * S::from_i32(velocity).unwrap())
            }
            motor::MotorControl::Position(position, velocity) => MotorControl::Position(
                Radians * S::from_f64(position.as_radians()).unwrap(),
                RotationsPerMinute * S::from_i32(velocity).unwrap(),
            ),
        }
    }
}

/// A motor plugged into a Smart Port.
#[derive(Debug, PartialEq)]
pub struct Motor {
    inner: motor::Motor,
}

impl Motor {
    /// The maximum voltage value that can be sent to a V5 [`Motor`].
    pub const V5_MAX_VOLTAGE: f64 = motor::Motor::V5_MAX_VOLTAGE;
    /// The maximum voltage value that can be sent to a EXP [`Motor`].
    pub const EXP_MAX_VOLTAGE: f64 = motor::Motor::EXP_MAX_VOLTAGE;

    /// The interval at which the Brain will send new packets to a [`Motor`].
    pub const WRITE_INTERVAL: Duration = motor::Motor::WRITE_INTERVAL;

    /// Creates a new 11W (V5) Smart Motor.
    ///
    /// See [`Motor::new_exp`] to create a 5.5W (EXP) Smart Motor.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Red, Direction::Forward);
    ///     assert!(motor.is_v5());
    ///     assert_eq!(motor.max_voltage().unwrap(), Motor::V5_MAX_VOLTAGE);
    /// }
    #[must_use]
    pub fn new(port: SmartPort, gearset: Gearset, direction: Direction) -> Self {
        Self {
            inner: motor::Motor::new(port, gearset, direction),
        }
    }
    /// Creates a new 5.5W (EXP) Smart Motor.
    ///
    /// See [`Motor::new`] to create a 11W (V5) Smart Motor.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new_exp(peripherals.port_1, Direction::Forward);
    ///     assert!(motor.is_exp());
    ///     assert_eq!(motor.max_voltage().unwrap(), Motor::EXP_MAX_VOLTAGE);
    /// }
    #[must_use]
    pub fn new_exp(port: SmartPort, direction: Direction) -> Self {
        Self {
            inner: motor::Motor::new_exp(port, direction),
        }
    }

    /// Sets the target that the motor should attempt to reach.
    ///
    /// This could be a voltage, velocity, position, or even brake mode.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_target(MotorControl::Voltage(5.0));
    ///     sleep(Duration::from_secs(1)).await;
    ///     let _ = motor.set_target(MotorControl::Brake(BrakeMode::Hold));
    /// }
    /// ```
    pub fn set_target<S: Scalar + ToPrimitive>(
        &mut self,
        target: MotorControl<S>,
    ) -> Result<(), MotorError> {
        self.inner.set_target(target.into())
    }

    /// Sets the motors target to a given [`BrakeMode`].
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.brake(BrakeMode::Hold);
    /// }
    /// ```
    pub fn brake(&mut self, mode: BrakeMode) -> Result<(), MotorError> {
        self.set_target(MotorControl::<u8>::Brake(mode))
    }

    /// Spins the motor at a target velocity.
    ///
    /// This velocity corresponds to different actual speeds in RPM depending on the gearset used for the motor.
    /// Velocity is held with an internal PID controller to ensure consistent speed, as opposed to setting the
    /// motor's voltage.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Spin a motor at 100 RPM:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_velocity(100);
    ///     sleep(Duration::from_secs(1)).await;
    /// }
    /// ```
    pub fn set_velocity<S: Scalar + ToPrimitive>(
        &mut self,
        velocity: AngularVelocity<S>,
    ) -> Result<(), MotorError> {
        self.set_target(MotorControl::Velocity(velocity))
    }

    /// Sets the motor's output voltage.
    ///
    /// This voltage value spans from -12 (fully spinning reverse) to +12 (fully spinning forwards) volts, and
    /// controls the raw output of the motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Give the motor full power:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut v5_motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let mut exp_motor = Motor::new_exp(peripherals.port_2, Direction::Forward);
    ///     let _ = v5_motor.set_voltage(v5_motor.max_voltage());
    ///     let _ = exp_motor.set_voltage(exp_motor.max_voltage());
    /// }
    /// ```
    ///
    /// Drive the motor based on a controller joystick:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let controller = peripherals.primary_controller;
    ///     loop {
    ///         let controller_state = controller.state().unwrap_or_default();
    ///         let voltage = controller_state.left_stick.x() * motor.max_voltage();
    ///         motor.set_voltage(voltage).unwrap();
    ///     }
    /// }
    /// ```
    pub fn set_voltage<S: Scalar + ToPrimitive>(
        &mut self,
        volts: Voltage<S>,
    ) -> Result<(), MotorError> {
        self.set_target(MotorControl::Voltage(volts))
    }

    /// Sets an absolute position target for the motor to attempt to reach.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    ///
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_position_target(Position::from_degrees(90.0), 200);
    /// }
    /// ```
    pub fn set_position_target<S: Scalar + ToPrimitive>(
        &mut self,
        position: Angle<S>,
        velocity: AngularVelocity<S>,
    ) -> Result<(), MotorError> {
        self.set_target(MotorControl::Position(position, velocity))
    }

    /// Changes the output velocity for a profiled movement (motor_move_absolute or motor_move_relative).
    ///
    /// This will have no effect if the motor is not following a profiled movement.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     // Set the motor's target to a Position so that changing the velocity isn't a noop.
    ///     let _ = motor.set_target(MotorControl::Position(Position::from_degrees(90.0), 200));
    ///     let _ = motor.set_profiled_velocity(100).unwrap();
    /// }
    /// ```
    pub fn set_profiled_velocity<S: Scalar + ToPrimitive>(
        &mut self,
        velocity: AngularVelocity<S>,
    ) -> Result<(), MotorError> {
        self.inner
            .set_profiled_velocity(velocity.to::<RotationsPerMinute>().to_i32().unwrap())
    }

    /// Returns the current [`MotorControl`] target that the motor is attempting to use.
    /// This value is set with [`Motor::set_target`].
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     motor.set_target(MotorControl::Brake(BrakeMode::Hold));
    ///     let target = motor.target();
    ///     assert_eq!(target, MotorControl::Brake(BrakeMode::Hold));
    /// }
    #[must_use]
    pub fn target<S: Scalar>(&self) -> MotorControl<S> {
        self.inner.target().into()
    }

    /// Sets the gearset of an 11W motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    /// - A [`MotorError::SetGearsetExp`] is returned if the motor is a 5.5W EXP Smart Motor, which has no swappable gearset.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     // This must be a V5 motor
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///
    ///     // Set the motor to use the red gearset
    ///     motor.set_gearset(Gearset::Red).unwrap();
    /// }
    /// ```
    pub fn set_gearset(&mut self, gearset: Gearset) -> Result<(), MotorError> {
        self.inner.set_gearset(gearset)
    }

    /// Returns the gearset of the motor
    ///
    /// For 5.5W motors, this will always be returned as [`Gearset::Green`].
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the gearset of a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// fn print_gearset(motor: &Motor) {
    ///     let Ok(gearset) = motor.gearset() else {
    ///         println!("Failed to get gearset. Is this an EXP motor?");
    ///         return;
    ///     };
    ///     match motor.gearset() {
    ///         Gearset::Green => println!("Motor is using the green gearset"),
    ///         Gearset::Red => println!("Motor is using the red gearset"),
    ///         Gearset::Blue => println!("Motor is using the blue gearset"),
    ///    }
    /// }
    ///
    pub fn gearset(&self) -> Result<Gearset, MotorError> {
        self.inner.gearset()
    }

    /// Returns the type of the motor.
    /// This does not check the hardware, it simply returns the type that the motor was created with.
    ///
    /// # Examples
    ///
    /// Match based on motor type:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// fn print_motor_type(motor: &Motor) {
    ///     match motor.motor_type() {
    ///         MotorType::Exp => println!("Motor is a 5.5W EXP Smart Motor"),
    ///         MotorType::V5 => println!("Motor is an 11W V5 Smart Motor"),
    ///     }
    /// }
    /// ```
    #[must_use]
    pub const fn motor_type(&self) -> MotorType {
        self.inner.motor_type()
    }
    /// Returns `true` if the motor is a 5.5W (EXP) Smart Motor.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new_exp(peripherals.port_1, Direction::Forward);
    ///     if motor.is_exp() {
    ///         println!("Motor is a 5.5W EXP Smart Motor");
    ///     }
    /// }
    /// ```
    #[must_use]
    pub const fn is_exp(&self) -> bool {
        self.inner.is_exp()
    }
    /// Returns `true` if the motor is an 11W (V5) Smart Motor.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor:: new(peripherals.port_1, Gearset::Red, Direction::Forward);
    ///     if motor.is_v5() {
    ///         println!("Motor is an 11W V5 Smart Motor");
    ///     }
    /// }
    /// ```
    #[must_use]
    pub const fn is_v5(&self) -> bool {
        self.inner.is_v5()
    }

    /// Returns the maximum voltage for the motor based off of its [motor type](Motor::motor_type).
    ///
    /// # Examples
    ///
    /// Run a motor at max speed, agnostic of its type:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// fn run_motor_at_max_speed(motor: &mut Motor) {
    ///     motor.set_voltage(motor.max_voltage()).unwrap();
    /// }
    #[must_use]
    pub const fn max_voltage(&self) -> f64 {
        self.inner.max_voltage()
    }

    /// Returns the motor's estimate of its angular velocity.
    ///
    /// # Accuracy
    ///
    /// In some cases, this reported value may be noisy or innaccurate, especially for systems where accurate
    /// velocity control at high speeds is required (such as flywheels). If the accuracy of this value proves
    /// inadequate, you may opt to perform your own velocity calculations by differentiating [`Motor::position`]
    /// over the reported internal timestamp of the motor using [`Motor::timestamp`].
    ///
    /// > For more information about Smart motor velocity estimation, see [this article](https://sylvie.fyi/sylib/docs/db/d8e/md_module_writeups__velocity__estimation.html).
    ///
    /// # Note
    ///
    /// To get the current **target** velocity instead of the estimated velocity, use [`Motor::target`].
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Get the current velocity of a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///
    ///     println!("{:?}", motor.velocity().unwrap());
    /// }
    /// ```
    ///
    /// Calculate acceleration of a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///
    ///     let mut last_velocity = motor.velocity().unwrap();
    ///     let mut start_time = Instant::now();
    ///     loop {
    ///         let velocity = motor.velocity().unwrap();
    ///         // Make sure we don't divide by zero
    ///         let elapsed = start_time.elapsed().as_secs_f64() + 0.001;
    ///
    ///         // Calculate acceleration
    ///         let acceleration = (velocity - last_velocity) / elapsed;
    ///         println!("Velocity: {:.2} RPM, Acceleration: {:.2} RPM/s", velocity, acceleration);
    ///
    ///         last_velocity = velocity;
    ///         start_time = Instant::now();
    ///    }
    /// }
    /// ```
    pub fn velocity<S: Scalar>(&self) -> Result<AngularVelocity<S>, MotorError> {
        self.inner
            .velocity()
            .map(|velocity| RotationsPerMinute * S::from_f64(velocity).unwrap())
    }

    /// Returns the power drawn by the motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the power drawn by a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     loop {
    ///         println!("Power: {:.2}W", motor.power().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn power<S: Scalar>(&self) -> Result<Power<S>, MotorError> {
        self.inner
            .power()
            .map(|power| Watts * S::from_f64(power).unwrap())
    }

    //TODO
    // /// Returns the torque output of the motor.
    // ///
    // /// # Errors
    // ///
    // /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    // ///
    // /// # Examples
    // ///
    // /// Print the torque output of a motor:
    // ///
    // /// ```
    // /// use vexide::prelude::*;
    // ///
    // /// #[vexide::main]
    // /// async fn main(peripherals: Peripherals) {
    // ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    // ///     loop {
    // ///         println!("Torque: {:.2}Nm", motor.torque().unwrap());
    // ///         sleep(Motor::UPDATE_INTERVAL).await;
    // ///     }
    // /// }
    // /// ```
    // pub fn torque(&self) -> Result<f64, MotorError> {
    //     self.validate_port()?;
    //     Ok(unsafe { vexDeviceMotorTorqueGet(self.device) })
    // }

    /// Returns the voltage the motor is drawing in volts.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the voltage drawn by a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     loop {
    ///         println!("Voltage: {:.2}V", motor.voltage().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn voltage<S: Scalar>(&self) -> Result<Voltage<S>, MotorError> {
        self.inner
            .voltage()
            .map(|voltage| Volts * S::from_f64(voltage).unwrap())
    }

    /// Returns the current position of the motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the current position of a motor:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     loop {
    ///         println!("Position: {:?}", motor.position().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn position<S: Scalar>(&self) -> Result<Angle<S>, MotorError> {
        self.inner
            .position()
            .map(|position| Radians * S::from_f64(position.as_radians()).unwrap())
    }

    /// Returns the most recently recorded raw encoder tick data from the motor's IME
    /// along with a timestamp of the internal clock of the motor indicating when the
    /// data was recorded.
    ///
    /// # Errors
    ///
    /// Returns the electrical current draw of the motor in amps.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the current draw of a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     motor.set_voltage(motor.max_voltage()).unwrap();
    ///     loop {
    ///         println!("Current: {:.2}A", motor.current().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn current<S: Scalar>(&self) -> Result<Current<S>, MotorError> {
        self.inner
            .current()
            .map(|current| Amperes * S::from_f64(current).unwrap())
    }

    /// Returns the efficiency of the motor from a range of [0.0, 1.0].
    ///
    /// An efficiency of 1.0 means that the motor is moving electrically while
    /// drawing no electrical power, and an efficiency of 0.0 means that the motor
    /// is drawing power but not moving.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the efficiency of a motor:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage(motor.max_voltage())
    ///     loop {
    ///         println!("Efficiency: {:.2}", motor.efficiency().unwrap());
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn efficiency(&self) -> Result<f64, MotorError> {
        self.inner.efficiency()
    }

    /// Sets the current encoder position to zero without moving the motor.
    ///
    /// Analogous to taring or resetting the encoder to the current position.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Move the motor in increments of 10 degrees:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     loop {
    ///         motor.set_position_target(Position::from_degrees(10.0), 200).unwrap();
    ///         sleep(Duration::from_secs(1)).await;
    ///         motor.reset_position().unwrap();
    ///     }
    /// }
    /// ```
    pub fn reset_position(&mut self) -> Result<(), MotorError> {
        self.inner.reset_position()
    }

    /// Sets the current encoder position to the given position without moving the motor.
    ///
    /// Analogous to taring or resetting the encoder so that the new position is equal to the given position.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Set the current position of the motor to 90 degrees:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     motor.set_position(Position::from_degrees(90.0)).unwrap();
    /// }
    /// ```
    ///
    /// Reset the position of the motor to 0 degrees (analaogous to [`reset_position`](Motor::reset_position)):
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     motor.set_position(Position::from_degrees(0.0)).unwrap();
    /// }
    /// ```
    pub fn set_position<S: Scalar + ToPrimitive>(&mut self, position: Angle<S>) -> Result<(), MotorError> {
        self.inner.set_position(Position::from_radians(position.to::<Radians>().to_f64().unwrap()))
    }

    /// Sets the current limit for the motor in amps.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Limit the current draw of a motor to 2.5A:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_current_limit(2.5);
    /// }
    /// ```
    pub fn set_current_limit<S: Scalar + ToPrimitive>(&mut self, limit: Current<S>) -> Result<(), MotorError> {
        self.inner
            .set_current_limit(limit.to::<Amperes>().to_f64().unwrap())
    }

    /// Sets the voltage limit for the motor in volts.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Limit the voltage of a motor to 4V:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage_limit(4.0);
    ///     // Will appear as if the voltage was set to only 4V
    ///     let _ = motor.set_voltage(12.0);
    /// }
    /// ```
    pub fn set_voltage_limit<S: Scalar + ToPrimitive>(&mut self, limit: Voltage<S>) -> Result<(), MotorError> {
        self.inner
            .set_voltage_limit(limit.to::<Volts>().to_f64().unwrap())
    }

    /// Returns the current limit for the motor in amps.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the current limit of a motor:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     println!("Current Limit: {:.2}A", motor.current_limit().unwrap());
    /// }
    /// ```
    pub fn current_limit<S: Scalar>(&self) -> Result<Current<S>, MotorError> {
        self.inner
            .current_limit()
            .map(|limit| Amperes * S::from_f64(limit).unwrap())
    }

    /// Returns the voltage limit for the motor if one has been explicitly set.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print the voltage limit of a motor:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     println!("Voltage Limit: {:.2}V", motor.voltage_limit().unwrap());
    /// }
    /// ```
    pub fn voltage_limit<S: Scalar>(&self) -> Result<Voltage<S>, MotorError> {
        self.inner
            .voltage_limit()
            .map(|limit| Volts * S::from_f64(limit).unwrap())
    }

    /// Returns the internal temperature recorded by the motor in increments of 5 °C.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Turn off the motor if it gets too hot:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage(12.0);
    ///     loop {
    ///         if motor.temperature().unwrap() > 30.0 {
    ///             let _ = motor.brake(BrakeMode::Coast);
    ///         } else {
    ///             let _ = motor.set_voltage(12.0);
    ///         }
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn temperature<S: Scalar>(&self) -> Result<Temperature<S>, MotorError> {
        self.inner
            .temperature()
            .map(|temp| Celsius * S::from_f64(temp).unwrap())
    }

    /// Returns the status flags of a motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Check if a motor is "busy" (busy only occurs if communicating with the motor fails)
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// fn is_motor_busy(motor: &Motor) -> bool {
    ///     motor.status().unwrap().contains(MotorStatus::BUSY)
    /// }
    /// ```
    pub fn status(&self) -> Result<MotorStatus, MotorError> {
        self.inner.status()
    }

    /// Returns the fault flags of the motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Check if a motor is over temperature:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     loop {
    ///         let faults = motor.faults().unwrap();
    ///         println!("Faults: {:?}", faults);
    ///
    ///         if faults.contains(MotorFaults::OVER_TEMPERATURE) {
    ///             println!("Warning: Motor is over temperature");
    ///         }
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn faults(&self) -> Result<MotorFaults, MotorError> {
        self.inner.faults()
    }

    /// Returns `true` if the motor's over temperature flag is set.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Turn off the motor if it gets too hot:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage(12.0);
    ///     loop {
    ///         if let Ok(true) = motor.is_over_temperature() {
    ///             let _ = motor.brake(BrakeMode::Coast);
    ///         } else {
    ///             let _ = motor.set_voltage(12.0);
    ///         }
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn is_over_temperature(&self) -> Result<bool, MotorError> {
        self.inner.is_over_temperature()
    }

    /// Returns `true` if the motor's over-current flag is set.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print a warning if the motor draws too much current:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage(12.0);
    ///     loop {
    ///         if let Ok(true) = motor.is_over_current() {
    ///             println!("Warning: Motor is drawing too much current");
    ///         }
    ///         println!("Current: {:.2}A", motor.current().unwrap_or(0.0));
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn is_over_current(&self) -> Result<bool, MotorError> {
        self.inner.is_over_current()
    }

    /// Returns `true` if a H-bridge (motor driver) fault has occurred.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print a warning if the motor's H-bridge has a fault:
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage(12.0);
    ///     loop {
    ///         if let Ok(true) = motor.is_driver_fault() {
    ///             println!("Warning: Motor has a H-bridge fault");
    ///         }
    ///         println!("Current: {:.2}A", motor.current().unwrap_or(0.0));
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///     }
    /// }
    /// ```
    pub fn is_driver_fault(&self) -> Result<bool, MotorError> {
        self.inner.is_driver_fault()
    }

    /// Returns `true` if the motor's H-bridge has an over-current fault.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// Print a warning if it draws too much current:
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let _ = motor.set_voltage(12.0);
    ///     loop {
    ///         if let Ok(true) = motor.is_driver_over_current() {
    ///             println!("Warning: Motor is drawing too much current");
    ///         }
    ///         println!("Current: {:.2}A", motor.current().unwrap_or(0.0));
    ///         sleep(Motor::UPDATE_INTERVAL).await;
    ///    }
    /// }
    /// ```
    pub fn is_driver_over_current(&self) -> Result<bool, MotorError> {
        self.inner.is_driver_over_current()
    }

    /// Sets the motor to operate in a given [`Direction`].
    ///
    /// This determines which way the motor considers to be “forwards”. You can use the marking on the back of the
    /// motor as a reference:
    ///
    /// - When [`Direction::Forward`] is specified, positive velocity/voltage values will cause the motor to rotate
    ///   **with the arrow on the back**. Position will increase as the motor rotates **with the arrow**.
    /// - When [`Direction::Reverse`] is specified, positive velocity/voltage values will cause the motor to rotate
    ///   **against the arrow on the back**. Position will increase as the motor rotates **against the arrow**.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     motor.set_direction(Direction::Reverse).unwrap();
    /// }
    /// ```
    pub fn set_direction(&mut self, direction: Direction) -> Result<(), MotorError> {
        self.inner.set_direction(direction)
    }

    /// Returns the [`Direction`] of this motor.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// fn print_motor_direction(motor: &Motor) {
    ///     match motor.direction().unwrap() {
    ///         Direction::Forward => println!("Motor is set to forwards"),
    ///         Direction::Reverse => println!("Motor is set to reverse"),
    ///     }
    /// }
    /// ```
    pub fn direction(&self) -> Result<Direction, MotorError> {
        self.inner.direction()
    }

    /// Adjusts the internal tuning constants of the motor when using velocity control.
    ///
    /// # Hardware Safety
    ///
    /// Modifying internal motor control is **dangerous**, and can result in permanent hardware damage
    /// to Smart motors if done incorrectly. Use these functions entirely at your own risk.
    ///
    /// VEX has chosen not to disclose the default constants used by Smart motors, and currently
    /// has no plans to do so. As such, the units and finer details of [`MotorTuningConstants`] are not
    /// well-known or understood, as we have no reference for what these constants should look
    /// like.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let constants = MotorTuningConstants {
    ///         kf: 0.0,
    ///         kp: 0.0,
    ///         ki: 0.0,
    ///         kd: 0.0,
    ///         filter: 0.0,
    ///         integral_limit: 0.0,
    ///         tolerance: 0.0,
    ///         sample_rate: 0.0,
    ///     };
    ///     motor.set_velocity_tuning_constants(constants).unwrap();
    /// }
    /// ```
    #[cfg(feature = "dangerous_motor_tuning")]
    pub fn set_velocity_tuning_constants(
        &mut self,
        constants: MotorTuningConstants,
    ) -> Result<(), MotorError> {
        self.inner.set_velocity_tuning_constants(constants)
    }

    /// Adjusts the internal tuning constants of the motor when using position control.
    ///
    /// # Hardware Safety
    ///
    /// Modifying internal motor control is **dangerous**, and can result in permanent hardware damage
    /// to Smart motors if done incorrectly. Use these functions entirely at your own risk.
    ///
    /// VEX has chosen not to disclose the default constants used by Smart motors, and currently
    /// has no plans to do so. As such, the units and finer details of [`MotorTuningConstants`] are not
    /// well-known or understood, as we have no reference for what these constants should look
    /// like.
    ///
    /// # Errors
    ///
    /// - A [`MotorError::Port`] error is returned if a motor device is not currently connected to the Smart Port.
    ///
    /// # Examples
    ///
    /// ```
    /// use vexide::prelude::*;
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     let mut motor = Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward);
    ///     let constants = MotorTuningConstants {
    ///         kf: 0.0,
    ///         kp: 0.0,
    ///         ki: 0.0,
    ///         kd: 0.0,
    ///         filter: 0.0,
    ///         integral_limit: 0.0,
    ///         tolerance: 0.0,
    ///         sample_rate: 0.0,
    ///     };
    ///     motor.set_position_tuning_constants(constants).unwrap();
    /// }
    /// ```
    #[cfg(feature = "dangerous_motor_tuning")]
    pub fn set_position_tuning_constants(
        &mut self,
        constants: MotorTuningConstants,
    ) -> Result<(), MotorError> {
        self.inner.set_position_tuning_constants(constants)
    }
}
