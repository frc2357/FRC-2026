package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;

public class ShooterTuningSubsystem implements Sendable {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

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
  public double rpsTolerance = 0.1;

  private SparkBaseConfig m_motorconfig = SHOOTER.MOTOR_CONFIG_LEFT;
  // private ShooterCurveTuner m_curveTuner; TODO: implement later

  private AngularVelocity m_targetVelocity = Units.RotationsPerSecond.of(0);

  public ShooterTuningSubsystem() {
    m_motorLeft = new SparkMax(CAN_ID.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);

    m_motorRight = new SparkMax(
      CAN_ID.RIGHT_SHOOTER_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_motorRight.configure(
      SHOOTER.MOTOR_CONFIG_RIGHT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_PIDController = m_motorLeft.getClosedLoopController();
    m_encoder = m_motorLeft.getEncoder();

    Preferences.initDouble("shooterP", P);
    Preferences.initDouble("shooterI", I);
    Preferences.initDouble("shooterD", D);
    Preferences.initDouble("shooterStaticFF", staticFF);
    Preferences.initDouble("shooterVelocityFF", velocityFF);
    Preferences.initDouble("shooterAccelerationFF", accelerationFF);
    Preferences.initDouble(
      "shooterMaxVelocity",
      maxVelocity.in(RotationsPerSecond)
    );
    Preferences.initDouble(
      "shooterMaxAcceleration",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    Preferences.initDouble("shooterRpsTolerance", rpsTolerance);

    P = Preferences.getDouble("shooterP", P);
    I = Preferences.getDouble("shooterI", I);
    D = Preferences.getDouble("shooterD", D);
    staticFF = Preferences.getDouble("shooterStaticFF", staticFF);
    velocityFF = Preferences.getDouble("shooterVelocityFF", velocityFF);
    accelerationFF = Preferences.getDouble(
      "shooterAccelerationFF",
      accelerationFF
    );
    maxVelocity = RotationsPerSecond.of(
      Preferences.getDouble(
        "shooterMaxVelocity",
        maxVelocity.in(RotationsPerSecond)
      )
    );
    maxAcceleration = RotationsPerSecondPerSecond.of(
      Preferences.getDouble(
        "shooterMaxAcceleration",
        maxAcceleration.in(RotationsPerSecondPerSecond)
      )
    );
    rpsTolerance = Preferences.getDouble("shooterRpsTolerance", rpsTolerance);

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Shooter P", P);
    SmartDashboard.putNumber("Shooter I", I);
    SmartDashboard.putNumber("Shooter D", D);
    SmartDashboard.putNumber("Shooter Static FF", staticFF);
    SmartDashboard.putNumber("Shooter Velocity FF", velocityFF);
    SmartDashboard.putNumber("Shooter Acceleration FF", accelerationFF);

    SmartDashboard.putNumber(
      "Shooter MaxVel RPS",
      maxVelocity.in(RotationsPerSecond)
    );
    SmartDashboard.putNumber(
      "Shooter MaxAccel RPS",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    SmartDashboard.putNumber("RPS Tolerance", rpsTolerance);
    SmartDashboard.putNumber("Motor Velocity RPS", m_encoder.getVelocity());
    SmartDashboard.putNumber("Shooter Target RPS", 0);

    SmartDashboard.putBoolean("Is At Target", isAtTargetSpeed());
    SmartDashboard.putNumber("Voltage", m_motorLeft.getBusVoltage());
    SmartDashboard.putBoolean(
      "Shooter Running",
      !getVelocity().isNear(RotationsPerSecond.zero(), rpsTolerance)
    );
    SmartDashboard.putData("Save shooter Config", this);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pid(P, I, D);
    m_motorconfig.closedLoop.feedForward.sva(
      staticFF,
      velocityFF,
      accelerationFF
    );

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond))
      .cruiseVelocity(maxVelocity.in(RotationsPerSecond))
      .allowedProfileError(rpsTolerance);

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Shooter P", 0);
    double newI = SmartDashboard.getNumber("Shooter I", 0);
    double newD = SmartDashboard.getNumber("Shooter D", 0);
    double newStaticFF = SmartDashboard.getNumber("Shooter Static FF", 0);
    double newVelocityFF = SmartDashboard.getNumber("Shooter Velocity FF", 0);
    double newAccelerationFF = SmartDashboard.getNumber(
      "Shooter Acceleration FF",
      0
    );

    double newMaxVelocity = SmartDashboard.getNumber("Shooter MaxVel RPS", 0);
    double newMaxAcceleration = SmartDashboard.getNumber(
      "Shooter MaxAccel RPS",
      0
    );
    double newRpsTolerance = SmartDashboard.getNumber(
      "RPS Tolerance",
      rpsTolerance
    );

    SmartDashboard.putNumber(
      "Motor Velocity RPS",
      getVelocity().in(RotationsPerSecond)
    );

    SmartDashboard.putNumber("Voltage", m_motorLeft.getBusVoltage());
    SmartDashboard.putBoolean(
      "Shooter Running",
      !getVelocity().isNear(RotationsPerSecond.zero(), rpsTolerance)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetSpeed());

    m_targetVelocity = RotationsPerSecond.of(
      SmartDashboard.getNumber(
        "Shooter Target RPS",
        m_targetVelocity.in(RotationsPerSecond)
      )
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
      newRpsTolerance != rpsTolerance
    ) {
      P = newP;
      I = newI;
      D = newD;
      staticFF = newStaticFF;
      velocityFF = newVelocityFF;
      accelerationFF = newAccelerationFF;
      maxVelocity = RotationsPerSecond.of(newMaxVelocity);
      maxAcceleration = RotationsPerSecondPerSecond.of(newMaxAcceleration);
      rpsTolerance = newRpsTolerance;
      updatePIDs();
    }
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motorLeft.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless speed) {
    m_motorLeft.set(speed.times(SHOOTER.AXIS_MAX_SPEED).in(Value));
  }

  public void stop() {
    m_motorLeft.stopMotor();
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(m_encoder.getVelocity());
  }

  public void setTargetVelocity(AngularVelocity targetVelocity) {
    m_PIDController.setSetpoint(
      targetVelocity.in(RotationsPerSecond),
      ControlType.kMAXMotionVelocityControl
    );
  }

  public boolean isAtVel(AngularVelocity vel) {
    return vel.isNear(getVelocity(), rpsTolerance);
  }

  public boolean isAtTargetSpeed() {
    return isAtVel(m_targetVelocity);
  }

  public void teleopPeriodic() {
    setTargetVelocity(m_targetVelocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("shooter");

    builder.addBooleanProperty(
      "Save Config",
      () -> false,
      value -> {
        Preferences.setDouble("shooterP", P);
        Preferences.setDouble("shooterI", I);
        Preferences.setDouble("shooterD", D);
        Preferences.setDouble("shooterStaticFF", staticFF);
        Preferences.setDouble("shooterVelocityFF", velocityFF);
        Preferences.setDouble("shooterAccelerationFF", accelerationFF);
        Preferences.setDouble(
          "shooterMaxVelocity",
          maxVelocity.in(RotationsPerSecond)
        );
        Preferences.setDouble(
          "shooterMaxAcceleration",
          maxAcceleration.in(RotationsPerSecondPerSecond)
        );
        Preferences.setDouble("shooterRpsTolerance", rpsTolerance);
      }
    );
  }
}
