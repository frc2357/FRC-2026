package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;
import frc.robot.util.Utility;

public class Shooter extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  private boolean m_isAtZero = false;

  private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );
  private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
    Double.NaN
  );

  public Shooter() {
    m_motorLeft = new SparkMax(CAN_ID.SHOOTER_LEFT_MOTOR, MotorType.kBrushless);
    m_motorRight = new SparkMax(
      CAN_ID.SHOOTER_RIGHT_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      SHOOTER.MOTOR_CONFIG_LEFT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_motorRight.configure(
      SHOOTER.MOTOR_CONFIG_LEFT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_PIDController = m_motorLeft.getClosedLoopController();

    m_encoder = m_motorLeft.getEncoder();
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motorLeft.set(percentOutput.in(Value));
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  public void setAxisSpeed(Dimensionless speed) {
    double m_speed = speed.times(SHOOTER.AXIS_MAX_SPEED).in(Value);
    m_motorLeft.set(m_speed);
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
  }

  private void setTargetRotations(Angle targetRotations) {
    m_targetRotations.mut_replace(targetRotations);
    m_PIDController.setReference(
      m_targetRotations.in(Units.Rotations),
      ControlType.kMAXMotionVelocityControl,
      ClosedLoopSlot.kSlot0, //
      SHOOTER.LEFT_MOTOR_ARB_F,
      ArbFFUnits.kVoltage
    );
  }

  public void setTargetDistance(Distance targetDistance) {
    setTargetRotations(distanceToRotations(targetDistance));
  }

  public void holdPosition() {
    m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    m_motorLeft.setVoltage(SHOOTER.HOLD_VOLTAGE);
  }

  public AngularVelocity getVelocity() {
    m_currentAngularVelocityHolder.mut_replace(
      m_encoder.getVelocity(),
      Units.RPM
    );
    return m_currentAngularVelocityHolder;
  }

  private Angle getRotations() {
    m_currentRotationsHolder.mut_replace(
      m_encoder.getPosition(),
      Units.Rotations
    );
    return m_currentRotationsHolder;
  }

  public Distance getDistance() {
    return rotationsToDistance(getRotations());
  }

  private boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
      getRotations(),
      m_targetRotations,
      SHOOTER.SMART_MOTION_ALLOWED_ERROR_ROTATIONS
    );
  }

  public boolean isAtTarget() {
    return isAtTargetRotations();
  }

  public boolean isGoingDown() {
    return m_targetRotations.lt(getRotations());
  }

  public boolean isStalling() {
    return m_motorLeft.getOutputCurrent() > SHOOTER.ZERO_STALL_AMPS;
  }

  public boolean isAtZero() {
    return m_isAtZero;
  }

  public void setZero() {
    m_encoder.setPosition(0); //
  }

  private Angle distanceToRotations(Distance distance) {
    return Units.Rotations.of(
      distance
        .div(SHOOTER.OUTPUT_PULLEY_CIRCUMFERENCE)
        .times(SHOOTER.GEAR_RATIO)
        .magnitude()
    );
  }

  private Distance rotationsToDistance(Angle rotations) {
    return (
      SHOOTER.OUTPUT_PULLEY_CIRCUMFERENCE.times(
        rotations.div(SHOOTER.GEAR_RATIO).in(Units.Rotations)
      )
    );
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "Shooter Calculated Distance",
      getDistance().in(Units.Inches)
    ); //

    SmartDashboard.putBoolean("Hall Effect", isAtZero()); //
  }
}
