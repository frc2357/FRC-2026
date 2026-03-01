package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;
import java.util.function.Supplier;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Shooter extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;

  private SmartMotorControllerConfig m_smartMotorControllerConfig;
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController m_sparkSmartMotorController;

  private final FlyWheelConfig m_shooterConfig;
  // Shooter Mechanism
  private FlyWheel m_shooter;

  public Shooter() {
    m_motorLeft = new SparkMax(CAN_ID.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    m_motorRight = new SparkMax(
      CAN_ID.RIGHT_SHOOTER_MOTOR,
      MotorType.kBrushless
    );

    m_motorRight.configure(
      SHOOTER.SHOOTER_BASE_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withVendorConfig(SHOOTER.SHOOTER_BASE_CONFIG)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
        SHOOTER.P,
        SHOOTER.I,
        SHOOTER.D,
        SHOOTER.MAX_ANGULAR_VELOCITY,
        SHOOTER.MAX_ANGULAR_ACCELERATION
      )
      .withSimClosedLoopController(
        SHOOTER.P,
        SHOOTER.I,
        SHOOTER.D,
        SHOOTER.MAX_ANGULAR_VELOCITY,
        SHOOTER.MAX_ANGULAR_ACCELERATION
      )
      // Feedforward Constants
      .withFeedforward(SHOOTER.FEEDFORWARD)
      .withSimFeedforward(SHOOTER.FEEDFORWARD)
      // Telemetry name and verbosity level
      .withTelemetry(SHOOTER.MOTOR_NETWORK_KEY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      .withGearing(SHOOTER.GEARING)
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(SHOOTER.STALL_LIMIT);

    m_sparkSmartMotorController = new SparkWrapper(
      m_motorLeft,
      DCMotor.getNEO(1),
      m_smartMotorControllerConfig
    );

    m_shooterConfig = new FlyWheelConfig(m_sparkSmartMotorController)
      // Diameter of the flywheel.
      .withDiameter(SHOOTER.DIAMETER)
      // Mass of the flywheel.
      .withMass(SHOOTER.MASS)
      // Maximum speed of the shooter.
      .withUpperSoftLimit(SHOOTER.MAX_ANGULAR_VELOCITY)
      .withSpeedometerSimulation()
      // Telemetry name and verbosity for the arm.
      .withTelemetry(SHOOTER.MECHANISM_NETWORK_KEY, TelemetryVerbosity.HIGH);

    m_shooter = new FlyWheel(m_shooterConfig);
  }

  /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {
    return m_shooter.getSpeed();
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    return m_shooter.run(speed).finallyDo(() -> this.stopMotor());
  }

  /**
   * Set the shooter velocity.
   *
   * @param supplier Speed supplier to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(Supplier<AngularVelocity> supplier) {
    return m_shooter.run(supplier).finallyDo(() -> this.stopMotor());
  }

  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {
    m_shooter.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  private Command set(double dutyCycle) {
    return m_shooter.set(dutyCycle).finallyDo(() -> this.stopMotor());
  }

  public Command setSpeed(Dimensionless speed) {
    return set(speed.in(Value));
  }

  public Command axisSpeed(Supplier<Dimensionless> axis) {
    return m_shooter
      .set(() -> axis.get().times(SHOOTER.AXIS_MAX_SPEED).in(Value))
      .finallyDo(() -> this.stopMotor());
  }

  public Command stopCommand() {
    return m_shooter.set(0);
  }

  public void stopMotor() {
    m_shooter.setDutyCycleSetpoint(0);
  }

  public Command waitUntilTargetVelocity() {
    var trigger = new Trigger(
      () ->
        m_sparkSmartMotorController
          .getMechanismSetpointVelocity()
          .isPresent() &&
        m_sparkSmartMotorController
          .getMechanismVelocity()
          .isNear(
            m_sparkSmartMotorController.getMechanismSetpointVelocity().get(),
            SHOOTER.SCORE_TOLERANCE.in(Value)
          )
    ).debounce(SHOOTER.STABLE_VELOCITY.in(Seconds), DebounceType.kRising);
    return Commands.waitUntil(trigger);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_shooter.simIterate();
  }
}
