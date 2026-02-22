package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;

public class HoodTuningSubsystem implements Sendable {

  private SparkMax m_motorLeft;
  private SparkClosedLoopController m_PIDController;
  private AbsoluteEncoder m_encoder;

  public double P = 0.01;
  public double I = 0;
  public double D = 0;
  public double staticFF = 0.12;
  public double velocityFF = 0.1225;
  public double accelerationFF = 0.1225;
  public AngularVelocity maxVelocity = RotationsPerSecond.of(77); // Max at free speed is ~96, 80% is 77
  public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(
    150
  );
  public double RotationsTolerance = 0.1;

  private SparkBaseConfig m_motorconfig = HOOD.MOTOR_CONFIG;

  private Angle m_targetAngle = Units.Rotations.of(0);

  public HoodTuningSubsystem() {
    m_motorLeft = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();
    m_encoder = m_motorLeft.getAbsoluteEncoder();

    Preferences.initDouble("hoodP", P);
    Preferences.initDouble("hoodI", I);
    Preferences.initDouble("hoodD", D);
    Preferences.initDouble("hoodStaticFF", staticFF);
    Preferences.initDouble("hoodVelocityFF", velocityFF);
    Preferences.initDouble("hoodAccelerationFF", accelerationFF);
    Preferences.initDouble(
      "hoodMaxVelocity",
      maxVelocity.in(RotationsPerSecond)
    );
    Preferences.initDouble(
      "hoodMaxAcceleration",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    Preferences.initDouble("hoodRotationsTolerance", RotationsTolerance);

    P = Preferences.getDouble("hoodP", P);
    I = Preferences.getDouble("hoodI", I);
    D = Preferences.getDouble("hoodD", D);
    staticFF = Preferences.getDouble("hoodStaticFF", staticFF);
    velocityFF = Preferences.getDouble("hoodVelocityFF", velocityFF);
    accelerationFF = Preferences.getDouble(
      "hoodAccelerationFF",
      accelerationFF
    );
    maxVelocity = RotationsPerSecond.of(
      Preferences.getDouble(
        "hoodMaxVelocity",
        maxVelocity.in(RotationsPerSecond)
      )
    );
    maxAcceleration = RotationsPerSecondPerSecond.of(
      Preferences.getDouble(
        "hoodMaxAcceleration",
        maxAcceleration.in(RotationsPerSecondPerSecond)
      )
    );
    RotationsTolerance = Preferences.getDouble(
      "hoodRotationsTolerance",
      RotationsTolerance
    );

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Hood P", P);
    SmartDashboard.putNumber("Hood I", I);
    SmartDashboard.putNumber("Hood D", D);
    SmartDashboard.putNumber("Hood Static FF", staticFF);
    SmartDashboard.putNumber("Hood Velocity FF", velocityFF);
    SmartDashboard.putNumber("Hood Acceleration FF", accelerationFF);

    SmartDashboard.putNumber(
      "Hood MaxVel RPS",
      maxVelocity.in(RotationsPerSecond)
    );
    SmartDashboard.putNumber(
      "Hood MaxAccel RPS",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    SmartDashboard.putNumber("Rotations Tolerance", RotationsTolerance);
    SmartDashboard.putNumber("Hood Target Degrees", 0);
    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));

    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());
    SmartDashboard.putNumber("Voltage", m_motorLeft.getBusVoltage());

    SmartDashboard.putData("Save hood Config", this);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    m_motorconfig.closedLoop.positionWrappingEnabled(true);
    m_motorconfig.closedLoop.pid(P, I, D);
    m_motorconfig.closedLoop.feedForward.sva(
      staticFF,
      velocityFF,
      accelerationFF
    );

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond))
      .cruiseVelocity(maxVelocity.in(RotationsPerSecond))
      .allowedProfileError(RotationsTolerance);

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Hood P", 0);
    double newI = SmartDashboard.getNumber("Hood I", 0);
    double newD = SmartDashboard.getNumber("Hood D", 0);
    double newStaticFF = SmartDashboard.getNumber("Hood Static FF", 0);
    double newVelocityFF = SmartDashboard.getNumber("Hood Velocity FF", 0);
    double newAccelerationFF = SmartDashboard.getNumber(
      "Hood Acceleration FF",
      0
    );

    double newMaxVelocity = SmartDashboard.getNumber("Hood MaxVel RPS", 0);
    double newMaxAcceleration = SmartDashboard.getNumber(
      "Hood MaxAccel RPS",
      0
    );
    double newRotationsTolerance = SmartDashboard.getNumber(
      "Rotations Tolerance",
      RotationsTolerance
    );

    SmartDashboard.putNumber("Voltage", m_motorLeft.getBusVoltage());

    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());

    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));
    SmartDashboard.putNumber("Degrees", getAngle().in(Degrees));
    SmartDashboard.putNumber("VELOCITY RPM", getVelocity().in(RPM));

    SmartDashboard.putNumber("PID Setpoint", m_PIDController.getSetpoint());

    m_targetAngle = Degrees.of(
      SmartDashboard.getNumber("Hood Target Degrees", m_targetAngle.in(Degrees))
    );

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newStaticFF != staticFF ||
      newVelocityFF != velocityFF ||
      newAccelerationFF != accelerationFF ||
      newMaxVelocity != maxVelocity.in(RotationsPerSecond) ||
      newMaxAcceleration != maxAcceleration.in(RotationsPerSecondPerSecond) ||
      newRotationsTolerance != RotationsTolerance
    ) {
      P = newP;
      I = newI;
      D = newD;
      staticFF = newStaticFF;
      velocityFF = newVelocityFF;
      accelerationFF = newAccelerationFF;
      maxVelocity = RotationsPerSecond.of(newMaxVelocity);
      maxAcceleration = RotationsPerSecondPerSecond.of(newMaxAcceleration);
      RotationsTolerance = newRotationsTolerance;
      updatePIDs();
    }
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motorLeft.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless speed) {
    double m_speed = speed.times(HOOD.AXIS_MAX_SPEED).in(Value);
    m_motorLeft.set(m_speed);
  }

  public void stop() {
    m_motorLeft.stopMotor();
  }

  public Angle getAngle() {
    return Rotations.of(m_encoder.getPosition());
  }

  public AngularVelocity getVelocity() {
    return RPM.of(m_encoder.getVelocity());
  }

  public void setTargetAngle(Angle targetAngle) {
    m_PIDController.setSetpoint(
      targetAngle.in(Rotations),
      ControlType.kMAXMotionPositionControl
    );
  }

  public boolean isAtAngle(Angle vel) {
    return vel.isNear(getAngle(), RotationsTolerance);
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
        Preferences.setDouble("hoodVelocityFF", velocityFF);
        Preferences.setDouble("hoodAccelerationFF", accelerationFF);
        Preferences.setDouble(
          "hoodMaxVelocity",
          maxVelocity.in(RotationsPerSecond)
        );
        Preferences.setDouble(
          "hoodMaxAcceleration",
          maxAcceleration.in(RotationsPerSecondPerSecond)
        );
        Preferences.setDouble("hoodRotationsTolerance", RotationsTolerance);
      }
    );
  }
}
