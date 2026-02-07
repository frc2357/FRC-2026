package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;

public class Hood extends SubsystemBase {

  private SparkMax m_motor;
  private ProfiledPIDController m_PIDController;
  private RelativeEncoder m_encoder;

  private boolean m_isAtZero = false;

  private MutAngle m_targetRotations = Rotations.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = RPM.mutable(
    Double.NaN
  );
  private MutAngle m_currentRotationsHolder = Rotations.mutable(Double.NaN);

  public Hood() {
    m_motor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      HOOD.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_encoder = m_motor.getEncoder();
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    axisSpeed.times(HOOD.AXIS_MAX_SPEED);
    setSpeed(axisSpeed);
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public void holdPosition() {
    m_targetRotations.mut_replace(Double.NaN, Rotations);
    m_motor.setVoltage(HOOD.HOLD_VOLTAGE);
  }

  private Angle getRotations() {
    m_currentRotationsHolder.mut_replace(m_encoder.getPosition(), Rotations);
    return m_currentRotationsHolder;
  }

  public Angle getAngle() {
    return m_currentRotationsHolder;
  }

  private boolean isAtTargetRotations() {
    return Utility.isWithinTolerance(
      getRotations(),
      m_targetRotations,
      HOOD.SMART_MOTION_ALLOWED_ROTATIONS
    );
  }

  public boolean isAtZero() {
    return m_isAtZero;
  }

  private Angle angleToRotations(Angle angle) {
    return getRotations(); //Do calculations later
  }

  private Angle rotationsToAngle(Angle rotations) {
    return getRotations(); //
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("", getAngle().in(Degrees));
    SmartDashboard.putBoolean("not hall effect?", isAtZero());
  }
}
