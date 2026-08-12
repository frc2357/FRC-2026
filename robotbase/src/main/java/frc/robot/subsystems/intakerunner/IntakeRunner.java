package frc.robot.subsystems.intakerunner;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_RUNNER;

public class IntakeRunner extends SubsystemBase {

  private TalonFX m_leftMotor = new TalonFX(
    CAN_ID.LEFT_INTAKE_MOTOR,
    CANBus.roboRIO()
  );

  private TalonFX m_rightMotor = new TalonFX(
    CAN_ID.RIGHT_INTAKE_MOTOR,
    CANBus.roboRIO()
  );

  private final IntakeRunnerSim m_sim;

  public IntakeRunner() {
    m_leftMotor.getConfigurator().apply(Constants.INTAKE_RUNNER.MOTOR_CONFIG);
    m_rightMotor.getConfigurator().apply(Constants.INTAKE_RUNNER.MOTOR_CONFIG);

    m_rightMotor.setControl(
      new Follower(CAN_ID.LEFT_INTAKE_MOTOR, MotorAlignmentValue.Opposed)
    );

    m_sim = RobotBase.isSimulation()
      ? new IntakeRunnerSim(m_leftMotor, m_rightMotor)
      : null;
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_leftMotor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(INTAKE_RUNNER.AXIS_MAX_SPEED);
    setSpeed(m_speed);
  }

  public void stop() {
    m_leftMotor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "Intake Runner Motor Velocity (RPM)",
      m_leftMotor.getRotorVelocity().getValue().in(RPM)
    );
    SmartDashboard.putNumber(
      "Intake Runner Flywheel Velocity (RPM)",
      m_sim.getVelocityRPM()
    );
    SmartDashboard.putNumber(
      "Intake Runner Current Draw (A)",
      m_sim.getCurrentDrawAmps()
    );
  }
}
