package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.FEEDER;
import frc.robot.Constants.SHOOTER;
import java.util.function.Supplier;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.local.SparkWrapper;

public class Feeder extends SubsystemBase {

  private SparkMax m_motor;

  private SmartMotorControllerConfig m_smartMotorControllerConfig;
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController m_sparkSmartMotorController;

  private final FlyWheelConfig m_wheelConfig;
  // Shooter Mechanism
  private FlyWheel m_wheel;

  public Feeder() {
    m_motor = new SparkMax(CAN_ID.FEEDER_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      FEEDER.FEEDER_BASE_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withVendorConfig(FEEDER.FEEDER_BASE_CONFIG)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
        FEEDER.P,
        FEEDER.I,
        FEEDER.D,
        FEEDER.MAX_ANGULAR_VELOCITY,
        FEEDER.MAX_ANGULAR_ACCELERATION
      )
      .withSimClosedLoopController(
        FEEDER.P,
        FEEDER.I,
        FEEDER.D,
        FEEDER.MAX_ANGULAR_VELOCITY,
        FEEDER.MAX_ANGULAR_ACCELERATION
      )
      // Feedforward Constants
      .withFeedforward(FEEDER.FEEDFORWARD)
      .withSimFeedforward(FEEDER.FEEDFORWARD)
      // Telemetry name and verbosity level
      .withTelemetry(
        FEEDER.MOTOR_NETWORK_KEY,
        Constants.ROBOT.MECHANISM_VERBOSITY
      )
      // Gearing from the motor rotor to final shaft.
      .withGearing(FEEDER.GEARING)
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(FEEDER.STALL_LIMIT)
      .withClosedLoopTolerance(Rotations.of(0.01));

    m_sparkSmartMotorController = new SparkWrapper(
      m_motor,
      DCMotor.getNEO(1),
      m_smartMotorControllerConfig
    );

    m_wheelConfig = new FlyWheelConfig(m_sparkSmartMotorController)
      // Diameter of the flywheel.
      .withDiameter(FEEDER.DIAMETER)
      // Mass of the flywheel.
      .withMass(FEEDER.MASS)
      // Maximum speed of the shooter.
      .withUpperSoftLimit(FEEDER.MAX_ANGULAR_VELOCITY)
      .withSpeedometerSimulation()
      // Telemetry name and verbosity for the arm.
      .withTelemetry(
        FEEDER.MECHANISM_NETWORK_KEY,
        Constants.ROBOT.MECHANISM_VERBOSITY
      );

    m_wheel = new FlyWheel(m_wheelConfig);
  }

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return m_wheel.getSpeed();
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_wheel.run(speed).finallyDo(() -> this.stopMotor());
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return m_wheel.run(speed).finallyDo(() -> this.stopMotor());
  }

  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {
    m_wheel.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  private Command set(double dutyCycle) {
    return m_wheel.set(dutyCycle).finallyDo(() -> this.stopMotor());
  }

  public Command setSpeed(Dimensionless speed) {
    return set(speed.in(Value));
  }

  public Command setSpeed(Supplier<Dimensionless> speed) {
    return m_wheel
      .set(() -> speed.get().in(Value))
      .finallyDo(() -> this.stopMotor());
  }

  public Command axisSpeed(Supplier<Dimensionless> axis) {
    return m_wheel
      .set(() -> axis.get().times(SHOOTER.AXIS_MAX_SPEED).in(Value))
      .finallyDo(() -> this.stopMotor());
  }

  public Command stopCommand() {
    return m_wheel.set(0);
  }

  public void stopMotor() {
    m_wheel.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_wheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_wheel.simIterate();
  }
}
