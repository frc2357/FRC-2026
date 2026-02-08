package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_PIVOT;

public class intakePivotTuningSubsystem {

  private SparkMax m_motor;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private MutAngle m_targetAngle = Units.Rotations.mutable(Double.NaN);
  private MutAngle m_currentMutAngleHolder = Units.Rotations.mutable(
    Double.NaN
  );

  public intakePivotTuningSubsystem() {
    m_motor = new SparkMax(CAN_ID.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      INTAKE_PIVOT.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_PIDController = m_motor.getClosedLoopController();
    m_encoder = m_motor.getEncoder();
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
    m_targetAngle.mut_replace(Double.NaN, Rotations);
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(INTAKE_PIVOT.AXIS_MAX_SPEED);
    setSpeed(m_speed);
    m_targetAngle.mut_replace(Double.NaN, Rotations);
  }

  public void stop() {
    m_targetAngle.mut_replace(Double.NaN, Rotations);
    m_motor.stopMotor();
  }

  public Angle getAngle() {
    m_currentMutAngleHolder.mut_replace(
      m_encoder.getPosition(),
      Units.Rotations
    );
    return m_currentMutAngleHolder;
  }

  public void setTargetPosition(Angle targetPosition) {
    m_targetAngle.mut_replace(m_targetAngle);
    m_PIDController.setSetpoint(
      m_targetAngle.in(Rotations),
      ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0
    );
  }

  public boolean isAtRotations(Angle Rotations) {
    return Rotations.isNear(getAngle(), INTAKE_PIVOT.ANGULAR_TOLERANCE);
  }

  public boolean isAtTargetAngle() {
    return isAtRotations(m_targetAngle);
  }

  public void periodic() {
    SmartDashboard.putNumber("Rotations", getAngle().in(Rotations));
    SmartDashboard.putBoolean(
      "Intake Pivot Running",
      getAngle().gt(Rotations.of(500))
    );
  }
}
