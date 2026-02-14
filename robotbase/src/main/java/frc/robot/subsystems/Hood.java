package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;
import frc.robot.Constants.INTAKE_PIVOT;
import java.util.function.Supplier;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {

  private SparkMax m_motor;

  private SmartMotorControllerConfig m_smartMotorControllerConfig;
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController m_sparkSmartMotorController;

  private final PivotConfig m_hoodConfig;
  // HOOD Mechanism
  private Pivot m_hood;

  public Hood() {
    m_motor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);

    m_smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withVendorConfig(HOOD.HOOD_BASE_CONFIG)
      // Feedback Constants (PID Constants)
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
      )
      // Feedforward Constants
      .withFeedforward(HOOD.FEEDFORWARD)
      .withSimFeedforward(HOOD.FEEDFORWARD)
      // Telemetry name and verbosity level
      .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(HOOD.GEARING)
      // Motor properties to prevent over currenting.
      .withStatorCurrentLimit(HOOD.STALL_LIMIT);

    m_sparkSmartMotorController = new SparkWrapper(
      m_motor,
      DCMotor.getNEO(1),
      m_smartMotorControllerConfig
    );

    m_hoodConfig = new PivotConfig(m_sparkSmartMotorController)
      //both mass and length in a single function, no other implemntation is currently avalible
      .withMOI(INTAKE_PIVOT.LENGTH, INTAKE_PIVOT.MASS)
      .withSoftLimits(
        INTAKE_PIVOT.SOFT_LOWER_ANGLE,
        INTAKE_PIVOT.SOFT_UPPER_ANGLE
      )
      .withHardLimit(
        INTAKE_PIVOT.HARD_LOWER_ANGLE,
        INTAKE_PIVOT.HARD_UPPER_ANGLE
      )
      .withStartingPosition(INTAKE_PIVOT.STARTING_ANGLE)
      // Mass of the flywheel.
      // Maximum speed of the hood.
      // Telemetry name and verbosity for the arm.
      .withTelemetry(HOOD.NETWORK_KEY, TelemetryVerbosity.HIGH);

    m_hood = new Pivot(m_hoodConfig);
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
    return m_hood.run(angle).finallyDo(() -> this.stopMotor());
  }

  /**
   * Set the hood angle setpoint.
   *
   * @param speed Speed to set
   */
  public void setAngularSetpoint(AngularVelocity speed) {
    m_hood.setMechanismVelocitySetpoint(speed);
  }

  /**
   * Set the dutycycle of the hood.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  private Command set(double dutyCycle) {
    return m_hood
      .set(dutyCycle)
      //.alongWith(new InstantCommand(() -> System.out.println("Setting")))
      .finallyDo(() -> this.stopMotor());
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_hood.simIterate();
  }
}
