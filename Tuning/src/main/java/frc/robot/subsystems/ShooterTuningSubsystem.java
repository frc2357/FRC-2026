package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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

  private TalonFX m_motorLeft;
  private TalonFX m_motorRight;

  public double P = 0.004;
  public double I = 0;
  public double D = 0;
  public double staticFF = 0.14;
  public double velocityFF = 0.1255;
  public double accelerationFF = 0.01;
  public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(
    150
  );
  public double rpsTolerance = 0.01;

  private TalonFXConfiguration m_motorconfig = SHOOTER.MOTOR_CONFIG;
  // private ShooterCurveTuner m_curveTuner; TODO: implement later

  private AngularVelocity m_targetVelocity = Units.RotationsPerSecond.of(0);

  private MotionMagicVelocityVoltage m_velocityRequest =
    new MotionMagicVelocityVoltage(0);

  public ShooterTuningSubsystem() {
    m_motorLeft = new TalonFX(CAN_ID.LEFT_SHOOTER_MOTOR, CANBus.roboRIO());

    m_motorRight = new TalonFX(CAN_ID.RIGHT_SHOOTER_MOTOR, CANBus.roboRIO());

    m_motorLeft.getConfigurator().apply(m_motorconfig);
    m_motorRight.getConfigurator().apply(m_motorconfig);

    m_motorRight.setControl(
      new Follower(CAN_ID.LEFT_SHOOTER_MOTOR, MotorAlignmentValue.Opposed)
    );

    Preferences.initDouble("shooterP", P);
    Preferences.initDouble("shooterI", I);
    Preferences.initDouble("shooterD", D);
    Preferences.initDouble("shooterStaticFF", staticFF);
    Preferences.initDouble("shooterVelocityFF", velocityFF);
    Preferences.initDouble("shooterAccelerationFF", accelerationFF);
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
      "Shooter MaxAccel RPS",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    SmartDashboard.putNumber("RPS Tolerance", rpsTolerance);
    SmartDashboard.putNumber(
      "Motor Velocity RPS",
      getVelocity().in(RotationsPerSecond)
    );
    SmartDashboard.putNumber("Shooter Target RPS", 0);

    SmartDashboard.putBoolean("Is At Target", isAtTargetSpeed());
    SmartDashboard.putNumber(
      "Voltage",
      m_motorLeft.getMotorVoltage().getValue().in(Volts)
    );
    SmartDashboard.putBoolean(
      "Shooter Running",
      !getVelocity().isNear(RotationsPerSecond.zero(), rpsTolerance)
    );
    SmartDashboard.putData("Save shooter Config", this);
  }

  public void updatePIDs() {
    // set slot 0 gains
    var slot0Configs = m_motorconfig.Slot0;
    slot0Configs.kS = staticFF;
    slot0Configs.kV = velocityFF;
    slot0Configs.kA = accelerationFF;
    slot0Configs.kP = P;
    slot0Configs.kI = I;
    slot0Configs.kD = D;

    // set Motion Magic settings
    var motionMagicConfigs = m_motorconfig.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = maxAcceleration.in(
      RotationsPerSecondPerSecond
    );
    motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

    m_motorLeft.getConfigurator().apply(m_motorconfig);
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

    SmartDashboard.putNumber(
      "Voltage",
      m_motorLeft.getMotorVoltage().getValue().in(Volts)
    );
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
      newMaxAcceleration != maxAcceleration.in(RotationsPerSecondPerSecond) ||
      newRpsTolerance != rpsTolerance
    ) {
      P = newP;
      I = newI;
      D = newD;
      staticFF = newStaticFF;
      velocityFF = newVelocityFF;
      accelerationFF = newAccelerationFF;
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
    return m_motorLeft.getVelocity().getValue();
  }

  public void setTargetVelocity(AngularVelocity targetVelocity) {
    m_motorLeft.setControl(m_velocityRequest.withVelocity(targetVelocity));
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
          "shooterMaxAcceleration",
          maxAcceleration.in(RotationsPerSecondPerSecond)
        );
        Preferences.setDouble("shooterRpsTolerance", rpsTolerance);
      }
    );
  }
}
