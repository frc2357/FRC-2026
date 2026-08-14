package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SHOOTER;

public class Shooter extends SubsystemBase {

  private ShooterSim m_sim;

  private ClosedLoopSlot m_closedLoopSlot = SHOOTER.CLOSED_LOOP_SLOT;

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;

  private SparkClosedLoopController m_pidController;

  private MutAngularVelocity m_targetVelocity = RotationsPerSecond.mutable(
    Double.NaN
  );
  private MutAngularVelocity m_currentVelocityHolder =
    RotationsPerSecond.mutable(Double.NaN);

  public Shooter() {
    m_leftMotor = new SparkMax(CAN_ID.LEFT_SHOOTER_MOTOR, MotorType.kBrushless);
    m_rightMotor = new SparkMax(
      CAN_ID.RIGHT_SHOOTER_MOTOR,
      MotorType.kBrushless
    );

    m_leftMotor.configure(
      SHOOTER.SHOOTER_BASE_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );
    m_rightMotor.configure(
      SHOOTER.RIGHT_MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kPersistParameters
    );

    m_pidController = m_leftMotor.getClosedLoopController();

    if (RobotBase.isSimulation()) {
      m_sim = new ShooterSim(m_leftMotor, m_rightMotor);
      m_closedLoopSlot = SHOOTER.SIM_CLOSED_LOOP_SLOT;
    }
  }

  public void setVelocity(AngularVelocity velocity) {
    m_targetVelocity.mut_replace(velocity);
    m_pidController.setSetpoint(
      m_targetVelocity.in(RPM),
      ControlType.kVelocity,
      m_closedLoopSlot
    );
  }

  public void set(double dutyCycle) {
    m_leftMotor.set(dutyCycle);
  }

  public void set(Dimensionless speed) {
    set(speed.in(Value));
  }

  public void axisSpeed(Dimensionless speed) {
    set(speed.times(SHOOTER.AXIS_MAX_SPEED));
  }

  public void stepAxisSpeed(Dimensionless speed) {
    set(
      Math.floor(
          speed.times(SHOOTER.AXIS_MAX_SPEED).in(Value) / SHOOTER.STEP_AXIS_STEP
        ) *
        SHOOTER.STEP_AXIS_STEP
    );
  }

  public void stop() {
    m_leftMotor.set(0);
  }

  public Trigger isAtInitialTargetVelocity() {
    return new Trigger(() ->
      RPM.of(m_leftMotor.getEncoder().getVelocity()).isNear(
        m_targetVelocity,
        SHOOTER.INITIAL_SCORE_TOLERANCE
      )
    ).debounce(SHOOTER.STABLE_VELOCITY.in(Seconds), DebounceType.kRising);
  }

  public Trigger isAtContinuousTargetVelocity() {
    return new Trigger(() ->
      RPM.of(m_leftMotor.getEncoder().getVelocity()).isNear(
        m_targetVelocity,
        SHOOTER.CONTINUOUS_SCORE_TOLERANCE.in(Value)
      )
    ).debounce(SHOOTER.STABLE_VELOCITY.in(Seconds), DebounceType.kRising);
  }

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "Shooter Left Motor Velocity (RPM)",
      m_leftMotor.getEncoder().getVelocity()
    );
    SmartDashboard.putNumber(
      "Shooter Right Motor Velocity (RPM)",
      m_rightMotor.getEncoder().getVelocity()
    );
    SmartDashboard.putNumber(
      "Shooter Flywheel Velocity (RPM)",
      m_sim.getVelocityRPM()
    );
    SmartDashboard.putNumber(
      "Shooter Target Velocity (RPM)",
      m_targetVelocity.in(RPM)
    );
  }
}
