package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;
import frc.robot.Robot;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Hood extends SubsystemBase {

  private final SparkMax m_motor;

  private final SmartMotorControllerConfig m_smartMotorControllerConfig;
  private final SmartMotorController m_sparkSmartMotorController;

  private final PivotConfig m_hoodConfig;
  private final Pivot m_hood;

  private final SparkAbsoluteEncoder m_encoder1;
  private final SparkAbsoluteEncoder m_encoder2;

  private final EasyCRTConfig m_crtConfig;
  private final EasyCRT m_crtSolver;

  public Hood() {
    m_motor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);

    m_smartMotorControllerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withVendorConfig(HOOD.HOOD_BASE_CONFIG)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(
        HOOD.P,
        HOOD.I,
        HOOD.D,
        HOOD.MAX_ANGULAR_VELOCITY,
        HOOD.MAX_ANGULAR_ACCELERATION
      )
      .withSimClosedLoopController(
        HOOD.P,
        HOOD.I,
        HOOD.D,
        HOOD.MAX_ANGULAR_VELOCITY,
        HOOD.MAX_ANGULAR_ACCELERATION
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
      .withMOI(HOOD.LENGTH, HOOD.MASS)
      .withSoftLimits(HOOD.SOFT_LOWER_ANGLE, HOOD.SOFT_UPPER_ANGLE)
      .withHardLimit(HOOD.HARD_LOWER_ANGLE, HOOD.HARD_UPPER_ANGLE)
      .withStartingPosition(HOOD.STARTING_ANGLE)
      // Mass of the flywheel.
      // Maximum speed of the hood.
      // Telemetry name and verbosity for the arm.
      .withTelemetry(HOOD.NETWORK_KEY, TelemetryVerbosity.HIGH);

    m_hood = new Pivot(m_hoodConfig);

    m_encoder1 = m_motor.getAbsoluteEncoder();
    m_encoder2 = Robot.shooter.getSecondHoodEncoder();

    m_crtConfig = new EasyCRTConfig(() -> Units.Rotations.of(m_encoder1.getPosition()), () -> Units.Rotations.of(m_encoder2.getPosition()))
      .withAbsoluteEncoder1GearingStages(20,19,16,266)
      .withAbsoluteEncoder2GearingStages(16,266)
      .withMechanismRange(Units.Degrees.of(0), Units.Degrees.of(25));
    m_crtSolver = new EasyCRT(m_crtConfig);
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

  /**
   * The main purpose of this method is so we can tell if the hood is stationary
   * The CRT calculations can get messed up if the mechanism is moving
   */
  public AngularVelocity getMotorVelocity() {
    return Units.RPM.of(m_motor.getEncoder().getVelocity());
  }

  public boolean zero() {
    Optional<Angle> crtAngle = m_crtSolver.getAngleOptional();
    if (crtAngle.isPresent()) {
      System.out.println(crtAngle);
      // m_sparkSmartMotorController.setEncoderPosition(crtAngle.get());
      return true;
    } else {
      return false;
    }
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
