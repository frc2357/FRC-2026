package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;

public class HoodTuningSubsystem implements Sendable {

  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private SparkAbsoluteEncoder m_encoder;

  public double P = 50;
  public double I = 0;
  public double D = 0;
  public double staticFF = 0.1;
  public double gravityFF = 0.0;

  public Angle tolerance = Degrees.of(0.1);

  private SparkBaseConfig m_motorconfig = HOOD.MOTOR_CONFIG;

  private Angle m_targetAngle = Units.Rotations.of(0);

  public HoodTuningSubsystem() {
    m_motor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getAbsoluteEncoder();

    Preferences.initDouble("hoodP", P);
    Preferences.initDouble("hoodI", I);
    Preferences.initDouble("hoodD", D);
    Preferences.initDouble("hoodStaticFF", staticFF);
    Preferences.initDouble("hoodGravityFF", gravityFF);
    Preferences.initDouble("hoodTolerance", tolerance.in(Degrees));

    P = Preferences.getDouble("hoodP", P);
    I = Preferences.getDouble("hoodI", I);
    D = Preferences.getDouble("hoodD", D);
    staticFF = Preferences.getDouble("hoodStaticFF", staticFF);
    gravityFF = Preferences.getDouble("hoodGravityFF", gravityFF);
    tolerance = Degrees.of(
      Preferences.getDouble("hoodTolerance", tolerance.in(Degrees))
    );

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Hood P", P);
    SmartDashboard.putNumber("Hood I", I);
    SmartDashboard.putNumber("Hood D", D);
    SmartDashboard.putNumber("Hood Static FF", staticFF);
    SmartDashboard.putNumber("Hood Gravity FF", gravityFF);

    SmartDashboard.putNumber("Hood Degrees Tolerance", tolerance.in(Degrees));
    SmartDashboard.putNumber("Hood Target Degrees", 0);
    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));

    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());
    SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());

    SmartDashboard.putData("Save hood Config", this);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pid(P, I, D);
    m_motorconfig.closedLoop.feedForward.sg(staticFF, gravityFF);
    m_motorconfig.closedLoop.allowedClosedLoopError(
      tolerance.in(Rotations),
      ClosedLoopSlot.kSlot0
    );

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Hood P", 0);
    double newI = SmartDashboard.getNumber("Hood I", 0);
    double newD = SmartDashboard.getNumber("Hood D", 0);
    double newStaticFF = SmartDashboard.getNumber("Hood Static FF", 0);
    double newGravityFF = SmartDashboard.getNumber("Hood Gravity FF", 0);
    double newTolerance = SmartDashboard.getNumber("Hood Degrees Tolerance", 0);

    SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());

    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());

    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));
    SmartDashboard.putNumber("Hood Current Degrees", getAngle().in(Degrees));
    SmartDashboard.putNumber(
      "Hood Velocity RPS",
      getVelocity().in(RotationsPerSecond)
    );

    SmartDashboard.putNumber("PID Setpoint", m_PIDController.getSetpoint());

    m_targetAngle = Degrees.of(
      SmartDashboard.getNumber("Hood Target Degrees", m_targetAngle.in(Degrees))
    );

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newStaticFF != staticFF ||
      newGravityFF != gravityFF ||
      newTolerance != tolerance.in(Degrees)
    ) {
      P = newP;
      I = newI;
      D = newD;
      staticFF = newStaticFF;
      gravityFF = newGravityFF;
      tolerance = Degrees.of(newTolerance);
      System.out.println("Setting pids");
      updatePIDs();
    }
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless speed) {
    m_motor.set(speed.times(HOOD.AXIS_MAX_SPEED).in(Value));
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public Angle getAngle() {
    return Rotations.of(m_encoder.getPosition());
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(m_encoder.getVelocity());
  }

  public void setTargetAngle(Angle targetAngle) {
    m_PIDController.setSetpoint(
      targetAngle.in(Rotations),
      ControlType.kPosition
    );
  }

  public boolean isAtAngle(Angle vel) {
    return vel.isNear(getAngle(), tolerance);
  }

  public boolean isAtTargetAngle() {
    return isAtAngle(m_targetAngle);
  }

  public void teleopPeriodic() {
    setTargetAngle(m_targetAngle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("hood");

    builder.addBooleanProperty(
      "Save Config",
      () -> false,
      value -> {
        Preferences.setDouble("hoodP", P);
        Preferences.setDouble("hoodI", I);
        Preferences.setDouble("hoodD", D);
        Preferences.setDouble("hoodStaticFF", staticFF);
        Preferences.setDouble("hoodGravityFF", gravityFF);
        Preferences.setDouble("hoodTolerance", tolerance.in(Degrees));
      }
    );
  }
}
