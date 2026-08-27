package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_PIVOT;

public class IntakePivot extends SubsystemBase {

  private IntakePivotSim m_sim;

  private SparkMax m_motor;
  private SparkAbsoluteEncoder m_encoder;

  public IntakePivot() {
    m_motor = new SparkMax(CAN_ID.INTAKE_PIVOT_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      INTAKE_PIVOT.INTAKE_PIVOT_BASE_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_encoder = m_motor.getAbsoluteEncoder();

    if (RobotBase.isSimulation()) {
      m_sim = new IntakePivotSim(m_motor);
    }
  }

  public void set(double dutyCycle) {
    m_motor.set(dutyCycle);
  }

  public void set(Dimensionless speed) {
    set(speed.in(Value));
  }

  public void axisSpeed(Dimensionless speed) {
    set(speed.times(INTAKE_PIVOT.AXIS_MAX_SPEED));
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public Current getStatorCurrent() {
    return Amps.of(m_motor.getOutputCurrent());
  }

  public Trigger isIntakeCurrentStallingTrigger() {
    return new Trigger(() ->
      Amps.of(getStatorCurrent().abs(Amps)).gte(
        INTAKE_PIVOT.AMP_STALL_THRESHOLD
      )
    );
  }

  public Trigger isAbovePositionTrigger(Angle targetAngle) {
    return new Trigger(() ->
      Rotations.of(m_encoder.getPosition()).lte(targetAngle)
    );
  }

  public Trigger isBelowPositionTrigger(Angle targetAngle) {
    return new Trigger(() ->
      Rotations.of(m_encoder.getPosition()).gte(targetAngle)
    );
  }

  public Trigger isInDeployedPositionTrigger() {
    return isBelowPositionTrigger(
      INTAKE_PIVOT.INTAKE_DEPLOYED_ENCODER_ROTATIONS
    );
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "Intake Pivot Applied Output",
      m_motor.getAppliedOutput()
    );
    SmartDashboard.putNumber(
      "Intake Pivot Motor Velocity (RPM)",
      m_motor.getEncoder().getVelocity()
    );
    SmartDashboard.putNumber(
      "Intake Pivot Angle (Rotations)",
      Rotations.of(m_encoder.getPosition()).in(Rotations)
    );
    SmartDashboard.putBoolean(
      "Intake Pivot isIntakeCurrentStallingTrigger",
      isIntakeCurrentStallingTrigger().getAsBoolean()
    );
    SmartDashboard.putBoolean(
      "Intake Pivot isInDeployedPositionTrigger",
      isInDeployedPositionTrigger().getAsBoolean()
    );
  }
}
