package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.Constants.INTAKE_PIVOT;

public class IntakePivotTuningSubsystem implements Sendable {

  private TalonFX m_motor;

  public double P = 0.00;
  public double I = 0;
  public double D = 0;
  public double staticFF = 0;
  public double velocityFF = 0;
  public double accelerationFF = 0.0;
  public AngularVelocity maxVelocity = RotationsPerSecond.of(1);
  public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(
    0.2
  );
  public double positionTolerance = 0.02;

  private TalonFXConfiguration m_motorconfig = INTAKE_PIVOT.MOTOR_CONFIGURATION;

  private Angle m_targetPosition = Rotations.of(0);

  private MotionMagicVoltage m_motionRequest = new MotionMagicVoltage(0);

  public IntakePivotTuningSubsystem() {
    m_motor = new TalonFX(CAN_ID.INTAKE_PIVOT_MOTOR);

    m_motor.getConfigurator().apply(m_motorconfig);

    Preferences.initDouble("intakePivotP", P);
    Preferences.initDouble("intakePivotI", I);
    Preferences.initDouble("intakePivotD", D);
    Preferences.initDouble("intakePivotStaticFF", staticFF);
    Preferences.initDouble("intakePivotVelocityFF", velocityFF);
    Preferences.initDouble("intakePivotAccelerationFF", accelerationFF);
    Preferences.initDouble(
      "intakePivotMaxVelocity",
      maxVelocity.in(RotationsPerSecond)
    );
    Preferences.initDouble(
      "intakePivotMaxAcceleration",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    Preferences.initDouble("intakePivotRpsTolerance", positionTolerance);

    P = Preferences.getDouble("intakePivotP", P);
    I = Preferences.getDouble("intakePivotI", I);
    D = Preferences.getDouble("intakePivotD", D);
    staticFF = Preferences.getDouble("intakePivotStaticFF", staticFF);
    velocityFF = Preferences.getDouble("intakePivotVelocityFF", velocityFF);
    accelerationFF = Preferences.getDouble(
      "intakePivotAccelerationFF",
      accelerationFF
    );
    maxVelocity = RotationsPerSecond.of(
      Preferences.getDouble(
        "intakePivotMaxVelocity",
        maxVelocity.in(RotationsPerSecond)
      )
    );
    maxAcceleration = RotationsPerSecondPerSecond.of(
      Preferences.getDouble(
        "intakePivotMaxAcceleration",
        maxAcceleration.in(RotationsPerSecondPerSecond)
      )
    );
    positionTolerance = Preferences.getDouble(
      "intakePivotRpsTolerance",
      positionTolerance
    );

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("IntakePivot P", P);
    SmartDashboard.putNumber("IntakePivot I", I);
    SmartDashboard.putNumber("IntakePivot D", D);
    SmartDashboard.putNumber("IntakePivot Static FF", staticFF);
    SmartDashboard.putNumber("IntakePivot Velocity FF", velocityFF);
    SmartDashboard.putNumber("IntakePivot Acceleration FF", accelerationFF);

    SmartDashboard.putNumber(
      "IntakePivot MaxVel RPS",
      maxVelocity.in(RotationsPerSecond)
    );
    SmartDashboard.putNumber(
      "IntakePivot MaxAccel RPS",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    SmartDashboard.putNumber("RPS Tolerance", positionTolerance);
    SmartDashboard.putNumber(
      "Motor Velocity RPS",
      m_motor.getVelocity().getValue().in(RotationsPerSecond)
    );
    SmartDashboard.putNumber("Motor Position", getPosition().in(Rotations));
    SmartDashboard.putNumber("IntakePivot Target Position", 0);

    SmartDashboard.putBoolean("Is At Target", isAtTargetPosition());
    SmartDashboard.putNumber(
      "Voltage",
      m_motor.getMotorVoltage().getValueAsDouble()
    );
    SmartDashboard.putBoolean(
      "IntakePivot Running",
      !getVelocity().isNear(RotationsPerSecond.zero(), positionTolerance)
    );
    SmartDashboard.putData("Save intakePivot Config", this);
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
    motionMagicConfigs.MotionMagicCruiseVelocity = maxVelocity.in(
      RotationsPerSecond
    );
    motionMagicConfigs.MotionMagicAcceleration = maxAcceleration.in(
      RotationsPerSecondPerSecond
    );
    //motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    m_motor.getConfigurator().apply(m_motorconfig);
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("IntakePivot P", 0);
    double newI = SmartDashboard.getNumber("IntakePivot I", 0);
    double newD = SmartDashboard.getNumber("IntakePivot D", 0);
    double newStaticFF = SmartDashboard.getNumber("IntakePivot Static FF", 0);
    double newVelocityFF = SmartDashboard.getNumber(
      "IntakePivot Velocity FF",
      0
    );
    double newAccelerationFF = SmartDashboard.getNumber(
      "IntakePivot Acceleration FF",
      0
    );

    double newMaxVelocity = SmartDashboard.getNumber(
      "IntakePivot MaxVel RPS",
      0
    );
    double newMaxAcceleration = SmartDashboard.getNumber(
      "IntakePivot MaxAccel RPS",
      0
    );
    double newRpsTolerance = SmartDashboard.getNumber(
      "RPS Tolerance",
      positionTolerance
    );

    SmartDashboard.putNumber(
      "Motor Velocity RPS",
      getVelocity().in(RotationsPerSecond)
    );
    SmartDashboard.putNumber("Motor Position", getPosition().in(Rotations));

    SmartDashboard.putNumber(
      "Voltage",
      m_motor.getMotorVoltage().getValueAsDouble()
    );
    SmartDashboard.putBoolean(
      "IntakePivot Running",
      !getVelocity().isNear(RotationsPerSecond.zero(), positionTolerance)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetPosition());

    m_targetPosition = Rotations.of(
      SmartDashboard.getNumber(
        "IntakePivot Target Position",
        m_targetPosition.in(Rotations)
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
      newRpsTolerance != positionTolerance
    ) {
      P = newP;
      I = newI;
      D = newD;
      staticFF = newStaticFF;
      velocityFF = newVelocityFF;
      accelerationFF = newAccelerationFF;
      maxVelocity = RotationsPerSecond.of(newMaxVelocity);
      maxAcceleration = RotationsPerSecondPerSecond.of(newMaxAcceleration);
      positionTolerance = newRpsTolerance;
      updatePIDs();
    }
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless speed) {
    m_motor.set(speed.times(INTAKE_PIVOT.AXIS_MAX_SPEED).in(Value));
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public AngularVelocity getVelocity() {
    return m_motor.getVelocity().getValue();
  }

  public Angle getPosition() {
    return m_motor.getPosition().getValue();
  }

  public void setTargetPosition(Angle targetPosition) {
    m_motor.setControl(m_motionRequest.withPosition(targetPosition));
  }

  public boolean isAtPosition(Angle position) {
    return position.isNear(getPosition(), positionTolerance);
  }

  public boolean isAtTargetPosition() {
    return isAtPosition(m_targetPosition);
  }

  public void teleopPeriodic() {
    setTargetPosition(m_targetPosition);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("intakePivot");

    builder.addBooleanProperty(
      "Save Config",
      () -> false,
      value -> {
        Preferences.setDouble("intakePivotP", P);
        Preferences.setDouble("intakePivotI", I);
        Preferences.setDouble("intakePivotD", D);
        Preferences.setDouble("intakePivotStaticFF", staticFF);
        Preferences.setDouble("intakePivotVelocityFF", velocityFF);
        Preferences.setDouble("intakePivotAccelerationFF", accelerationFF);
        Preferences.setDouble(
          "intakePivotMaxVelocity",
          maxVelocity.in(RotationsPerSecond)
        );
        Preferences.setDouble(
          "intakePivotMaxAcceleration",
          maxAcceleration.in(RotationsPerSecondPerSecond)
        );
        Preferences.setDouble("intakePivotRpsTolerance", positionTolerance);
      }
    );
  }
}
