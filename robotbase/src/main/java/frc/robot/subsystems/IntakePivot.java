package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_PIVOT;
import java.util.function.Supplier;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivot extends SubsystemBase {

  private SparkMax m_motor;

  private SmartMotorControllerConfig m_smartMotorControllerConfig;
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController m_sparkSmartMotorController;

  private final ArmConfig m_armConfig;
  // INTAKE_PIVOT Mechanism
  private Arm m_arm;

  private AbsoluteEncoder m_encoder;

  public IntakePivot() {
    m_motor = new SparkMax(CAN_ID.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

    m_encoder = m_motor.getAbsoluteEncoder();
    m_smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withExternalEncoder(m_encoder)
      .withExternalEncoderInverted(INTAKE_PIVOT.ENCODER_INVERTED)
      .withUseExternalFeedbackEncoder(true)
      .withExternalEncoderZeroOffset(INTAKE_PIVOT.EXTERNAL_ENCODER_OFFSET)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withVendorConfig(INTAKE_PIVOT.INTAKE_PIVOT_BASE_CONFIG)
      .withFeedforward(INTAKE_PIVOT.FEEDFORWARD)
      .withSimFeedforward(INTAKE_PIVOT.FEEDFORWARD)
      // Telemetry name and verbosity level
      .withTelemetry(INTAKE_PIVOT.MOTOR_NETWORK_KEY, TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(INTAKE_PIVOT.GEARING)
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(INTAKE_PIVOT.STALL_LIMIT)
      .withClosedLoopController(
        INTAKE_PIVOT.P,
        INTAKE_PIVOT.I,
        INTAKE_PIVOT.D,
        INTAKE_PIVOT.MAX_ANGULAR_VELOCITY,
        INTAKE_PIVOT.MAX_ANGULAR_ACCELERATION
      )
      .withSimClosedLoopController(
        INTAKE_PIVOT.P,
        INTAKE_PIVOT.I,
        INTAKE_PIVOT.D,
        INTAKE_PIVOT.MAX_ANGULAR_VELOCITY,
        INTAKE_PIVOT.MAX_ANGULAR_ACCELERATION
      );
    m_sparkSmartMotorController = new SparkWrapper(
      m_motor,
      DCMotor.getNEO(1),
      m_smartMotorControllerConfig
    );

    m_armConfig = new ArmConfig(m_sparkSmartMotorController)
      // Diameter of the arm.
      // Mass of the arm.
      .withMass(INTAKE_PIVOT.MASS)
      .withLength(INTAKE_PIVOT.LENGTH)
      .withHardLimit(INTAKE_PIVOT.SIM_LOWER_ANGLE, INTAKE_PIVOT.SIM_UPPER_ANGLE)
      // Telemetry name and verbosity for the arm.
      .withTelemetry(
        INTAKE_PIVOT.MECHANISM_NETWORK_KEY,
        TelemetryVerbosity.HIGH
      );

    m_arm = new Arm(m_armConfig);
  }

  /**
   * Gets the current velocity of the intake_pivot.
   *
   * @return INTAKE_PIVOT velocity.
   */
  public Angle getAngle() {
    return m_arm.getAngle();
  }

  /**
   * Set the intake_pivot velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setAngle(Angle angle) {
    return m_arm.run(angle).finallyDo(() -> this.stopMotor());
  }

  /**
   * Set the intake_pivot velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setAngularSetpoint(Angle angle) {
    m_arm.setMechanismPositionSetpoint(angle);
  }

  /**
   * Set the dutycycle of the intake_pivot.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  private Command set(double dutyCycle) {
    return m_arm.set(dutyCycle).finallyDo(() -> this.stopMotor());
  }

  public Command setSpeed(Dimensionless speed) {
    return set(speed.in(Value));
  }

  public Command axisSpeed(Supplier<Dimensionless> axis) {
    return m_arm
      .set(() -> axis.get().times(INTAKE_PIVOT.AXIS_MAX_SPEED).in(Value))
      .finallyDo(() -> this.stopMotor());
  }

  public Command stopCommand() {
    return m_arm.set(0);
  }

  public void stopMotor() {
    m_arm.setDutyCycleSetpoint(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_arm.simIterate();
  }
}
