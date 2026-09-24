package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Dimensionless;
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

  public IntakeRunner() {
    m_leftMotor.getConfigurator().apply(Constants.INTAKE_RUNNER.MOTOR_CONFIG);
    m_rightMotor.getConfigurator().apply(Constants.INTAKE_RUNNER.MOTOR_CONFIG);

    m_rightMotor.setControl(
      new Follower(CAN_ID.LEFT_INTAKE_MOTOR, MotorAlignmentValue.Opposed)
    );
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
}
