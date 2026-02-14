package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
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
      .withVendorConfig(SHOOTER.SHOOTER_BASE_CONFIG)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(1, 0, 0)
      .withSimClosedLoopController(1, 0, 0)
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      // .withMotorInverted(false)
      // .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withFollowers(Pair.of(m_motorRight, true));

    m_sparkSmartMotorController = new SparkWrapper(
      m_motorLeft,
      DCMotor.getNEO(1),
      m_smartMotorControllerConfig
    );

    m_shooterConfig = new FlyWheelConfig(m_sparkSmartMotorController)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel.
      .withMass(Pounds.of(1))
      // Maximum speed of the shooter.
      .withUpperSoftLimit(RPM.of(1000))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

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
    return m_shooter.run(speed);
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
  public Command set(double dutyCycle) {
    return m_shooter
      .set(dutyCycle)
      .alongWith(new InstantCommand(() -> System.out.println("Setting")));
  }

  public Command axisSpeed(Supplier<Dimensionless> axis) {
    return m_shooter.set(() -> {
      System.out.println(axis.get().in(Value));
      return axis.get().times(SHOOTER.AXIS_MAX_SPEED).in(Value);
    });
  }

  public Command stop() {
    return m_shooter.set(0);
  }

  public Command test() {
    return Commands.run(() -> m_motorLeft.set(0.5));
  }
}
