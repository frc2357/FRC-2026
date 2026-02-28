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
import com.revrobotics.spark.SparkAbsoluteEncoder;
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
import frc.robot.Constants.INTAKE_PIVOT;

public class IntakePivotTuningSubsystem implements Sendable {

  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private SparkAbsoluteEncoder m_encoder;

  public double P = 0;
  public double I = 0;
  public double D = 0;
  public double staticFF = 0;
  public double velocityFF = 0;
  public double accelerationFF = 0;
  public double gravityCosFF = 0;
  public AngularVelocity maxVelocity = INTAKE_PIVOT.MAX_POSSIBLE_VELOCITY.times(
    0
  );
  public AngularAcceleration maxAcceleration = RotationsPerSecondPerSecond.of(
    0
  );
  public double rotationsTolerance = 0.1;

  private SparkBaseConfig m_motorconfig = INTAKE_PIVOT.MOTOR_CONFIG;

  private Angle m_targetAngle = Units.Rotations.of(0);

  public IntakePivotTuningSubsystem() {
    m_motor = new SparkMax(CAN_ID.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_PIDController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getAbsoluteEncoder();

    Preferences.initDouble("intakePivotP", P);
    Preferences.initDouble("intakePivotI", I);
    Preferences.initDouble("intakePivotD", D);
    Preferences.initDouble("intakePivotStaticFF", staticFF);
    Preferences.initDouble("intakePivotVelocityFF", velocityFF);
    Preferences.initDouble("intakePivotAccelerationFF", accelerationFF);
    Preferences.initDouble("intakePivotGravityCosFF", gravityCosFF);
    Preferences.initDouble(
      "intakePivotMaxVelocity",
      maxVelocity.in(RotationsPerSecond)
    );
    Preferences.initDouble(
      "intakePivotMaxAcceleration",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    Preferences.initDouble("intakePivotRotationsTolerance", rotationsTolerance);

    P = Preferences.getDouble("intakePivotP", P);
    I = Preferences.getDouble("intakePivotI", I);
    D = Preferences.getDouble("intakePivotD", D);
    staticFF = Preferences.getDouble("intakePivotStaticFF", staticFF);
    velocityFF = Preferences.getDouble("intakePivotVelocityFF", velocityFF);
    accelerationFF = Preferences.getDouble(
      "intakePivotAccelerationFF",
      accelerationFF
    );
    gravityCosFF = Preferences.getDouble(
      "intakePivotGravityCosFF",
      gravityCosFF
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
    rotationsTolerance = Preferences.getDouble(
      "intakePivotRotationsTolerance",
      rotationsTolerance
    );

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Intake Pivot P", P);
    SmartDashboard.putNumber("Intake Pivot I", I);
    SmartDashboard.putNumber("Intake Pivot D", D);
    SmartDashboard.putNumber("Intake Pivot Static FF", staticFF);
    SmartDashboard.putNumber("Intake Pivot Velocity FF", velocityFF);
    SmartDashboard.putNumber("Intake Pivot Acceleration FF", accelerationFF);
    SmartDashboard.putNumber("Intake Pivot Gravity Cos FF", gravityCosFF);

    SmartDashboard.putNumber(
      "Intake Pivot MaxVel RPS",
      maxVelocity.in(RotationsPerSecond)
    );
    SmartDashboard.putNumber(
      "Intake Pivot MaxAccel RPS",
      maxAcceleration.in(RotationsPerSecondPerSecond)
    );
    SmartDashboard.putNumber("Rotations Tolerance", rotationsTolerance);
    SmartDashboard.putNumber("Intake Pivot Target Degrees", 0);
    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));

    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());
    SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());

    SmartDashboard.putData("Save intakePivot Config", this);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pid(P, I, D);
    m_motorconfig.closedLoop.feedForward
      .sva(staticFF, velocityFF, accelerationFF)
      .kCos(gravityCosFF);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(maxAcceleration.in(RotationsPerSecondPerSecond))
      .cruiseVelocity(maxVelocity.in(RotationsPerSecond))
      .allowedProfileError(rotationsTolerance);

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void updateDashboard() {
    double newP = SmartDashboard.getNumber("Intake Pivot P", 0);
    double newI = SmartDashboard.getNumber("Intake Pivot I", 0);
    double newD = SmartDashboard.getNumber("Intake Pivot D", 0);
    double newStaticFF = SmartDashboard.getNumber("Intake Pivot Static FF", 0);
    double newVelocityFF = SmartDashboard.getNumber(
      "Intake Pivot Velocity FF",
      0
    );
    double newAccelerationFF = SmartDashboard.getNumber(
      "Intake Pivot Acceleration FF",
      0
    );
    double newGravityCosFF = SmartDashboard.getNumber(
      "Intake Pivot Gravity Cos FF",
      0
    );

    double newMaxVelocity = SmartDashboard.getNumber(
      "Intake Pivot MaxVel RPS",
      0
    );
    double newMaxAcceleration = SmartDashboard.getNumber(
      "Intake Pivot MaxAccel RPS",
      0
    );
    double newRotationsTolerance = SmartDashboard.getNumber(
      "Rotations Tolerance",
      rotationsTolerance
    );

    SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());

    SmartDashboard.putBoolean("Is At Target", isAtTargetAngle());

    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));
    SmartDashboard.putNumber(
      "Intake Pivot Current Degrees",
      getAngle().in(Degrees)
    );
    SmartDashboard.putNumber(
      "Intake Pivot Velocity RPS",
      getVelocity().in(RotationsPerSecond)
    );

    SmartDashboard.putNumber("PID Setpoint", m_PIDController.getSetpoint());

    SmartDashboard.putNumber(
      "computed max rps",
      INTAKE_PIVOT.MAX_POSSIBLE_VELOCITY.in(RotationsPerSecond)
    );

    m_targetAngle = Degrees.of(
      SmartDashboard.getNumber(
        "Intake Pivot Target Degrees",
        m_targetAngle.in(Degrees)
      )
    );

    if (
      newP != P ||
      newI != I ||
      newD != D ||
      newStaticFF != staticFF ||
      newVelocityFF != velocityFF ||
      newAccelerationFF != accelerationFF ||
      newGravityCosFF != gravityCosFF ||
      newMaxVelocity != maxVelocity.in(RotationsPerSecond) ||
      newMaxAcceleration != maxAcceleration.in(RotationsPerSecondPerSecond) ||
      newRotationsTolerance != rotationsTolerance
    ) {
      P = newP;
      I = newI;
      D = newD;
      staticFF = newStaticFF;
      velocityFF = newVelocityFF;
      accelerationFF = newAccelerationFF;
      gravityCosFF = newGravityCosFF;
      maxVelocity = RotationsPerSecond.of(newMaxVelocity);
      maxAcceleration = RotationsPerSecondPerSecond.of(newMaxAcceleration);
      rotationsTolerance = newRotationsTolerance;
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

  public Angle getAngle() {
    return Rotations.of(m_encoder.getPosition());
  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(m_encoder.getVelocity());
  }

  public void setTargetAngle(Angle targetAngle) {
    m_PIDController.setSetpoint(
      targetAngle.in(Rotations),
      ControlType.kMAXMotionPositionControl
    );
  }

  public boolean isAtAngle(Angle angle) {
    return angle.isNear(getAngle(), rotationsTolerance);
  }

  public boolean isAtTargetAngle() {
    return isAtAngle(m_targetAngle);
  }

  public void teleopPeriodic() {
    setTargetAngle(m_targetAngle);
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
        Preferences.setDouble("intakePivotGravityCosFF", gravityCosFF);
        Preferences.setDouble(
          "intakePivotMaxVelocity",
          maxVelocity.in(RotationsPerSecond)
        );
        Preferences.setDouble(
          "intakePivotMaxAcceleration",
          maxAcceleration.in(RotationsPerSecondPerSecond)
        );
        Preferences.setDouble(
          "intakePivotRotationsTolerance",
          rotationsTolerance
        );
      }
    );
  }
}
