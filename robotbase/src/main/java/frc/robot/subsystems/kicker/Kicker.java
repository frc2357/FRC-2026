package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.KICKER;

public class Kicker extends SubsystemBase {

  private SparkMax m_motor;

  private final KickerSim m_sim;

  public Kicker() {
    m_motor = new SparkMax(CAN_ID.KICKER_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      KICKER.KICKER_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_sim = RobotBase.isSimulation() ? new KickerSim(m_motor) : null;
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(KICKER.AXIS_MAX_SPEED);
    setSpeed(m_speed);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "Kicker Motor Velocity (RPM)",
      m_motor.getEncoder().getVelocity()
    );
    SmartDashboard.putNumber(
      "Kicker Flywheel Velocity (RPM)",
      m_sim.getVelocityRPM()
    );
    SmartDashboard.putNumber(
      "Kicker Current Draw (A)",
      m_sim.getCurrentDrawAmps()
    );
  }
}
