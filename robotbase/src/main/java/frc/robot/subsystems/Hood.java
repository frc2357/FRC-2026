package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;
import java.util.function.Supplier;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {

  private SparkMax m_motor;
  private SparkAbsoluteEncoder m_encoder;

  private SmartMotorControllerConfig m_smartMotorControllerConfig;
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController m_sparkSmartMotorController;

  private final PivotConfig m_hoodConfig;
  // HOOD Mechanism
  private Pivot m_hood;

  public Hood() {
    m_motor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);
    m_encoder = m_motor.getAbsoluteEncoder();

    m_smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withVendorConfig(HOOD.HOOD_BASE_CONFIG)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(HOOD.P, HOOD.I, HOOD.D)
      .withSimClosedLoopController(HOOD.P, HOOD.I, HOOD.D)
      // Feedforward Constants
      .withFeedforward(HOOD.FEEDFORWARD)
      .withSimFeedforward(HOOD.FEEDFORWARD)
      // Telemetry name and verbosity level
      .withTelemetry(
        HOOD.MOTOR_NETWORK_KEY,
        Constants.ROBOT.MECHANISM_VERBOSITY
      )
      // Gearing from the motor rotor to final shaft.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(HOOD.GEARING)
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(HOOD.STALL_LIMIT)
      .withExternalEncoder(m_encoder)
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderGearing(HOOD.ENCODER_GEARING)
      .withExternalEncoderZeroOffset(HOOD.ADJUSTED_ZERO_OFFSET)
      .withSoftLimit(HOOD.LOWER_ANGLE_LIMIT, HOOD.UPPER_ANGLE_LIMIT);

    m_sparkSmartMotorController = new SparkWrapper(
      m_motor,
      DCMotor.getNEO(1),
      m_smartMotorControllerConfig
    );

    m_hoodConfig = new PivotConfig(m_sparkSmartMotorController)
      //both mass and length in a single function, no other implemntation is currently avalible
      .withMOI(HOOD.LENGTH, HOOD.MASS)
      .withHardLimit(HOOD.LOWER_ANGLE_LIMIT, HOOD.UPPER_ANGLE_LIMIT)
      .withStartingPosition(HOOD.SIM_STARTING_POSITION)
      // Mass of the flywheel.
      // Telemetry name and verbosity for the arm.
      .withTelemetry(
        HOOD.MECHANISM_NETWORK_KEY,
        Constants.ROBOT.MECHANISM_VERBOSITY
      );

    m_hood = new Pivot(m_hoodConfig);

    // Account for YAMS assuming the feedback sensor returns velocity nativley in RPM
    // The CANANDMAG Helium Encoder 0.2 natively returns rotations per second
    // so the conversion factor should not be divided by 60 like YAMS is doing.
    SparkBaseConfig baseConfig =
      (SparkBaseConfig) m_sparkSmartMotorController.getMotorControllerConfig();
    baseConfig.absoluteEncoder.velocityConversionFactor(
      HOOD.ENCODER_GEARING.getRotorToMechanismRatio()
    );
    m_motor.configure(
      baseConfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  /**
   * Gets the current angle of the hood.
   *
   * @return Hood angle.
   */
  public Angle getAngle() {
    return m_hood.getAngle();
  }

  /**
   * Set the hood angle.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setAngle(Angle angle) {
    return m_hood.run(angle);
  }

  public Command setAngle(Supplier<Angle> angle) {
    return m_hood.run(angle);
  }

  public Command goHome() {
    return this.setAngle(HOOD.SETPOINTS.HOME);
  }

  /**
   * Set the hood angle setpoint.
   *
   * @param speed Speed to set
   */
  public void setAngleSetpoint(Angle angle) {
    m_hood.setMechanismPositionSetpoint(angle);
  }

  /**
   * Set the dutycycle of the hood.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  private Command set(double dutyCycle) {
    return m_hood.set(dutyCycle);
  }

  public Command setSpeed(Dimensionless speed) {
    return set(speed.in(Value));
  }

  public Command axisSpeed(Supplier<Dimensionless> axis) {
    return m_hood
      .set(() -> axis.get().times(HOOD.AXIS_MAX_SPEED).in(Value))
      .finallyDo(() -> this.stopMotor());
  }

  public Command stopCommand() {
    return m_hood.set(0);
  }

  public void stopMotor() {
    m_hood.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hood.updateTelemetry();
    // SmartDashboard.putNumber("hood current degree", getAngle().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_hood.simIterate();
  }
}
