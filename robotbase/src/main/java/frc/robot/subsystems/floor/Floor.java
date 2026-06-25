package frc.robot.subsystems.floor;

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
import frc.robot.Constants.FLOOR;

public class Floor extends SubsystemBase {

  private SparkMax m_motor;

  private final FloorSim m_sim;

  public Floor() {
    m_motor = new SparkMax(CAN_ID.FLOOR_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      FLOOR.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_sim = RobotBase.isSimulation() ? new FloorSim(m_motor) : null;
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    axisSpeed.times(FLOOR.AXIS_MAX_SPEED);
    setSpeed(axisSpeed);
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "Floor Motor Velocity (RPM)",
      m_motor.getEncoder().getVelocity()
    );
    SmartDashboard.putNumber(
      "Floor Flywheel Velocity (RPM)",
      m_sim.getVelocityRPM()
    );
    SmartDashboard.putNumber(
      "Floor Current Draw (A)",
      m_sim.getCurrentDrawAmps()
    );
  }
}
