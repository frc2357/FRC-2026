package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.OUTTAKE;

public class Outtake extends SubsystemBase {

  private SparkMax m_Motor;

  public Outtake() {
    m_Motor = new SparkMax(CAN_ID.OUTAKE_MOTOR, MotorType.kBrushless);

    m_Motor.configure(
      OUTTAKE.OUTTAKE_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_Motor.set(percentOutput.in(Value));
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    Dimensionless m_speed = axisSpeed.times(OUTTAKE.AXIS_MAX_SPEED);
    setSpeed(m_speed);
  }

  public void stop() {
    m_Motor.stopMotor();
  }
}
