package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.FEEDER;

public class Feeder extends SubsystemBase {

  private FeederSim m_sim;

  private ClosedLoopSlot m_closedLoopSlot = FEEDER.CLOSED_LOOP_SLOT;

  private SparkMax m_motor;

  private SparkClosedLoopController m_pidController;

  private MutAngularVelocity m_targetVelocity = RotationsPerSecond.mutable(
    Double.NaN
  );
  private MutAngularVelocity m_currentVelocityHolder =
    RotationsPerSecond.mutable(Double.NaN);

  public Feeder() {
    m_motor = new SparkMax(CAN_ID.FEEDER_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      FEEDER.FEEDER_BASE_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_pidController = m_motor.getClosedLoopController();

    if (RobotBase.isSimulation()) {
      m_sim = new FeederSim(m_motor);
      m_closedLoopSlot = FEEDER.SIM_CLOSED_LOOP_SLOT;
    }
  }

  public AngularVelocity getVelocity() {
    m_currentVelocityHolder.mut_replace(
      m_motor.getEncoder().getPosition(),
      RPM
    );
    return m_currentVelocityHolder;
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
    m_motor.set(dutyCycle);
  }

  public void set(Dimensionless speed) {
    set(speed.in(Value));
  }

  public void axisSpeed(Dimensionless speed) {
    set(speed.times(FEEDER.AXIS_MAX_SPEED));
  }

  public void stepAxisSpeed(Dimensionless speed) {
    set(
      Math.floor(
          speed.times(FEEDER.AXIS_MAX_SPEED).in(Value) / FEEDER.STEP_AXIS_STEP
        ) *
        FEEDER.STEP_AXIS_STEP
    );
  }

  public void stop() {
    m_motor.set(0);
  }

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "Feeder Motor Velocity (RPM)",
      m_motor.getEncoder().getPosition()
    );
    SmartDashboard.putNumber(
      "Feeder Flywheel Velocity (RPM)",
      m_sim.getVelocityRPM()
    );
    SmartDashboard.putNumber(
      "Feeder Target Velocity (RPM)",
      m_targetVelocity.in(RPM)
    );
  }
}
