package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.INTAKE_PIVOT;

public class IntakePivot extends SubsystemBase {

  private SparkMax m_Motor;

  public IntakePivot() {
    m_Motor = new SparkMax(CAN_ID.LEFT_INTAKE_MOTOR, MotorType.kBrushless);

    m_Motor.configure(
      INTAKE_PIVOT.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_Motor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(INTAKE_PIVOT.AXIS_MAX_SPEED);
    setSpeed(m_speed);
  }

  public void stop() {
    m_Motor.stopMotor();
  }
}
