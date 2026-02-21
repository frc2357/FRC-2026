package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_PIVOT;

public class IntakePivotTuningSubsystem extends SubsystemBase {

  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  public double MOTOR_kP = 0;
  public double MOTOR_kI = 0;
  public double MOTOR_kD = 0;
  public double MOTOR_kS = 0;
  public double MOTOR_kV = 0;
  public double MOTOR_kA = 0;
  public double MAX_VEL = 0;
  public double MAX_ACCEL = 0; //TODO: find actual values
  public double ANGLE_TOLERANCE = 100; //

  private SparkBaseConfig m_motorconfig = INTAKE_PIVOT.MOTOR_CONFIG;

  private MutAngle m_targetAngle = Units.Degree.mutable(Double.NaN);
  private MutAngle m_currentAngleHolder = Units.Degree.mutable(Double.NaN);

  public IntakePivotTuningSubsystem() {
    m_motor = new SparkMax(CAN_ID.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

    m_motor = new SparkMax(CAN_ID.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_PIDController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("IntakePivot P", MOTOR_kP);
    SmartDashboard.putNumber("IntakePivot I", MOTOR_kI);
    SmartDashboard.putNumber("IntakePivot D", MOTOR_kD);
    SmartDashboard.putNumber("IntakePivot kV", MOTOR_kV);
    SmartDashboard.putNumber("IntakePivot kS", MOTOR_kS);
    SmartDashboard.putNumber("IntakePivot kA", MOTOR_kA);

    SmartDashboard.putNumber("IntakePivot MaxVel", MAX_VEL);
    SmartDashboard.putNumber("IntakePivot MaxAccel", MAX_ACCEL);
    SmartDashboard.putNumber("Angle Tolerence", ANGLE_TOLERANCE);
    SmartDashboard.putNumber("Target Degrees", 0);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pid(MOTOR_kP, MOTOR_kI, MOTOR_kD);
    m_motorconfig.closedLoop.feedForward.sva(MOTOR_kS, MOTOR_kV, MOTOR_kA);

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(MAX_ACCEL)
      .cruiseVelocity(MAX_VEL)
      .allowedProfileError(ANGLE_TOLERANCE);

    m_motor.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    MOTOR_kP = SmartDashboard.getNumber("IntakePivot P", 0);
    MOTOR_kI = SmartDashboard.getNumber("IntakePivot I", 0);
    MOTOR_kD = SmartDashboard.getNumber("IntakePivot D", 0);
    MOTOR_kS = SmartDashboard.getNumber("IntakePivot kS", 0);
    MOTOR_kV = SmartDashboard.getNumber("IntakePivot kV", 0);
    MOTOR_kA = SmartDashboard.getNumber("IntakePivot kA", 0);
    MAX_VEL = SmartDashboard.getNumber("IntakePivot MaxVel", 0);
    MAX_ACCEL = SmartDashboard.getNumber("IntakePivot MaxAccel", 0);
    ANGLE_TOLERANCE = SmartDashboard.getNumber(
      "Angle Tolerance",
      ANGLE_TOLERANCE
    );
    SmartDashboard.putNumber("Degrees", getAngle().in(Degrees));
    SmartDashboard.putNumber("setpoint", m_PIDController.getSetpoint());
    SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());
    SmartDashboard.putBoolean(
      "IntakePivot Running",
      !getAngle().isNear(Degrees.zero(), ANGLE_TOLERANCE)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetSpeed());
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
    m_targetAngle.mut_replace(Double.NaN, Units.Degrees);
  }

  public void setAxisSpeed(Dimensionless speed) {
    double m_speed = speed.times(INTAKE_PIVOT.AXIS_MAX_SPEED).in(Value);
    m_motor.set(m_speed);
    m_targetAngle.mut_replace(Double.NaN, Units.Degrees);
  }

  public void stop() {
    m_motor.stopMotor();
    m_targetAngle.mut_replace(Double.NaN, Units.Degrees);
  }

  public Angle getAngle() {
    m_currentAngleHolder.mut_replace(m_encoder.getVelocity(), Units.Degrees);
    return m_currentAngleHolder;
  }

  public void setTargetAngle(Angle targetVelocity) {
    m_targetAngle.mut_replace(targetVelocity);
    m_PIDController.setSetpoint(
      m_targetAngle.in(Degrees),
      ControlType.kMAXMotionVelocityControl
    );
  }

  public boolean isAtAngle(Angle Degrees) {
    return Degrees.isNear(getAngle(), ANGLE_TOLERANCE);
  }

  public boolean isAtTargetSpeed() {
    return isAtAngle(m_targetAngle);
  }

  public void periodic() {
    updateDashboard();
    updatePIDs();
    setTargetAngle(
      Degrees.of(
        SmartDashboard.getNumber("Target Degrees", m_targetAngle.in(Degrees))
      )
    );
  }
}
