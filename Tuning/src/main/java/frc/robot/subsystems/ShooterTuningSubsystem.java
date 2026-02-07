package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;

public class ShooterTuningSubsystem implements Sendable {

  private SparkMax m_motorLeft;
  private SparkMax m_motorRight;
  private ProfiledPIDController m_PIDController;
  private RelativeEncoder m_encoder;

  // private ShooterCurveTuner m_curveTuner; TODO: implement later

  private MutAngularVelocity m_targetVelocity = Units.RPM.mutable(Double.NaN);
  private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
    Double.NaN
  );

  private double P = 0;
  private double I = 0;
  private double D = 0;
  private double kG = 0;
  private double maxVel = 0;
  private double maxAccel = 0; //find values??

  private SparkBaseConfig m_motorconfig = Constants.SHOOTER.MOTOR_CONFIG_LEFT;

  public ShooterTuningSubsystem() {
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

    displayDashboard();
    updatePIDs();
  }

  public void displayDashboard() {
    SmartDashboard.putNumber("Shooter P", P);
    SmartDashboard.putNumber("Shooter I", I);
    SmartDashboard.putNumber("Shooter D", D);
    SmartDashboard.putNumber("Shooter kG", kG);
    SmartDashboard.putNumber("Shooter maxVel", maxVel);
    SmartDashboard.putNumber("Shooter maxAccel", maxAccel);
  }

  public void updatePIDs() {
    m_motorLeft.configure(
      m_motorconfig,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void updateDashboard() {
    P = Preferences.getDouble("shooterP", 0);
    I = Preferences.getDouble("elevatorI", 0);
    D = Preferences.getDouble("elevatorD", 0);
    kG = Preferences.getDouble("elevatorKG", 0);
    maxVel = Preferences.getDouble("elevatorMaxVel", 0);
    maxAccel = Preferences.getDouble("elevatorMaxAcc", 0);

    updatePIDs();
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

  public void periodic() {
    SmartDashboard.putNumber("RPM", getVelocity().in(RPM));
    SmartDashboard.putNumber(
      "Set point",
      m_PIDController.getSetpoint().velocity
    );
    SmartDashboard.putNumber("", m_motorLeft.getBusVoltage());
    SmartDashboard.putBoolean("Shooter Running", getVelocity().gt(RPM.of(500)));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
      "Unimplemented method 'initSendable'"
    );
  }

  // public double[] getShooterCurveRow() {
  //  return m_curveTuner.getSelectedRow();
  //} TODO: implement later
}
