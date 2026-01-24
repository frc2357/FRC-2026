package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE;

public class Intake extends SubsystemBase {

  private SparkMax m_motor;
  private SparkMax m_motor2;

  public Intake() {
    m_motor = new SparkMax(CAN_ID.INTAKE_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      INTAKE.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_motor2 = new SparkMax(CAN_ID.INTAKE_MOTOR, MotorType.kBrushless);

    m_motor2.configure(
      INTAKE.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(INTAKE.AXIS_MAX_SPEED);
    setSpeed(m_speed);
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
