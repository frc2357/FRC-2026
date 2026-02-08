package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;

public class ShooterTuningSubsystem extends SubsystemBase {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private SparkClosedLoopController m_PIDController;
  private RelativeEncoder m_encoder;

  public double LEFT_MOTOR_kP = 0;
  public double LEFT_MOTOR_kI = 0;
  public double LEFT_MOTOR_kD = 0;
  public double LEFT_MOTOR_kS = 0;
  public double LEFT_MOTOR_kV = 0;
  public double LEFT_MOTOR_kA = 0;
  public double MAX_VEL = 0;
  public double MAX_ACCEL = 0; //TODO: find actual values
  public double RPM_TOLERANCE = 100; //

  private SparkBaseConfig m_motorconfig = SHOOTER.MOTOR_CONFIG_LEFT;
  // private ShooterCurveTuner m_curveTuner; TODO: implement later

  private MutAngularVelocity m_targetVelocity = Units.RPM.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );

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

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Shooter P", LEFT_MOTOR_kP);
    SmartDashboard.putNumber("Shooter I", LEFT_MOTOR_kI);
    SmartDashboard.putNumber("Shooter D", LEFT_MOTOR_kD);
    SmartDashboard.putNumber("Shooter kV", LEFT_MOTOR_kV);
    SmartDashboard.putNumber("Shooter kS", LEFT_MOTOR_kS);
    SmartDashboard.putNumber("Shooter kA", LEFT_MOTOR_kA);

    SmartDashboard.putNumber("Shooter MaxVel", MAX_VEL);
    SmartDashboard.putNumber("Shooter MaxAccel", MAX_ACCEL);
    SmartDashboard.putNumber("RPM Tolerence", RPM_TOLERANCE);
    SmartDashboard.putNumber("Target RPM", 0);
  }

  public void updatePIDs() {
    m_motorconfig.closedLoop.pid(LEFT_MOTOR_kP, LEFT_MOTOR_kI, LEFT_MOTOR_kD);
    m_motorconfig.closedLoop.feedForward.sva(
      LEFT_MOTOR_kS,
      LEFT_MOTOR_kV,
      LEFT_MOTOR_kA
    );

    m_motorconfig.closedLoop.maxMotion
      .maxAcceleration(MAX_ACCEL)
      .cruiseVelocity(MAX_VEL)
      .allowedProfileError(RPM_TOLERANCE);

    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    LEFT_MOTOR_kP = SmartDashboard.getNumber("Shooter P", 0);
    LEFT_MOTOR_kI = SmartDashboard.getNumber("Shooter I", 0);
    LEFT_MOTOR_kD = SmartDashboard.getNumber("Shooter D", 0);
    LEFT_MOTOR_kS = SmartDashboard.getNumber("Shooter kS", 0);
    LEFT_MOTOR_kV = SmartDashboard.getNumber("Shooter kV", 0);
    LEFT_MOTOR_kA = SmartDashboard.getNumber("Shooter kA", 0);
    MAX_VEL = SmartDashboard.getNumber("Shooter MaxVel", 0);
    MAX_ACCEL = SmartDashboard.getNumber("Shooter MaxAccel", 0);
    RPM_TOLERANCE = SmartDashboard.getNumber("RPM Tolerance", RPM_TOLERANCE);
    SmartDashboard.putNumber("RPM", getVelocity().in(RPM));
    SmartDashboard.putNumber("setpoint", m_PIDController.getSetpoint());
    SmartDashboard.putNumber("Voltage", m_motorLeft.getBusVoltage());
    SmartDashboard.putBoolean(
      "Shooter Running",
      !getVelocity().isNear(RPM.zero(), RPM_TOLERANCE)
    );
    SmartDashboard.putBoolean("Is At Target", isAtTargetSpeed());
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

  public void setTargetVelocity(AngularVelocity targetVelocity) {
    m_targetVelocity.mut_replace(targetVelocity);
    m_PIDController.setSetpoint(
      m_targetVelocity.in(RPM),
      ControlType.kMAXMotionVelocityControl
    );
  }

  public boolean isAtRPM(AngularVelocity rpm) {
    return rpm.isNear(getVelocity(), RPM_TOLERANCE);
  }

  public boolean isAtTargetSpeed() {
    return isAtRPM(m_targetVelocity);
  }

  public void periodic() {
    updateDashboard();
    updatePIDs();
    setTargetVelocity(
      RPM.of(SmartDashboard.getNumber("Target RPM", m_targetVelocity.in(RPM)))
    );
  }
  // public double[] getShooterCurveRow() {
  //  return m_curveTuner.getSelectedRow();
  //} TODO: implement later
}
