package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;

public class Shooter extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private ProfiledPIDController m_PIDController;
  private RelativeEncoder m_encoder;

  // private ShooterCurveTuner m_curveTuner; TODO: implement later

  private MutAngularVelocity m_targetVelocity = Units.RPM.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );

  public Shooter() {
    m_motorLeft = new SparkMax(CAN_ID.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    m_motorRight = new SparkMax(
      CAN_ID.RIGHT_SHOOTER_MOTOR,
      MotorType.kBrushless
    );

    m_motorLeft.configure(
      SHOOTER.MOTOR_CONFIG_LEFT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
    m_motorRight.configure(
      SHOOTER.MOTOR_CONFIG_RIGHT,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_PIDController = new ProfiledPIDController(
      SHOOTER.LEFT_MOTOR_P,
      SHOOTER.LEFT_MOTOR_I,
      SHOOTER.LEFT_MOTOR_D,
      new TrapezoidProfile.Constraints(SHOOTER.MAX_VEL, SHOOTER.MAX_ACCEL)
    );

    m_encoder = m_motorLeft.getEncoder();
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motorLeft.set(percentOutput.in(Value));
    m_targetVelocity.mut_replace(Double.NaN, Units.RPM);
  }

  public void setAxisSpeed(Dimensionless speed) {
    double m_speed = speed.times(SHOOTER.AXIS_MAX_SPEED).in(Value);
    m_motorLeft.set(m_speed);
    m_targetVelocity.mut_replace(Double.NaN, Units.RPM);
  }

  public void stop() {
    m_motorLeft.stopMotor();
    m_targetVelocity.mut_replace(Double.NaN, Units.RPM);
  }

  public AngularVelocity getVelocity() {
    m_currentAngularVelocityHolder.mut_replace(
      m_encoder.getVelocity(),
      Units.RPM
    );
    return m_currentAngularVelocityHolder;
  }

  public void updateMotorPIDs() {
    m_motorLeft.setVoltage(
      m_PIDController.calculate(getVelocity().in(RPM), m_targetVelocity.in(RPM))
    );
  }

  public void setTargetVelocity(AngularVelocity targetVelocity) {
    m_targetVelocity.mut_replace(targetVelocity);
    m_PIDController.setGoal(m_targetVelocity.in(RPM));
  }

  public boolean isAtRPM(AngularVelocity rpm) {
    return rpm.isNear(getVelocity(), SHOOTER.RPM_TOLERANCE);
  }

  public boolean isAtTargetSpeed() {
    return isAtRPM(m_targetVelocity);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RPM", getVelocity().in(RPM));
    SmartDashboard.putBoolean("Shooter Running", getVelocity().gt(RPM.of(500)));
  }

  // public double[] getShooterCurveRow() {
  //  return m_curveTuner.getSelectedRow();
  //} TODO: implement later
}
