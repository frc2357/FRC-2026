package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.TUNNEL;

public class Tunnel extends SubsystemBase {

  private SparkMax m_motor;

  public Tunnel() {
    m_motor = new SparkMax(CAN_ID.TUNNEL_MOTOR, MotorType.kBrushless);
    m_motor.configure(
      TUNNEL.MOTOR_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );
  }

  public void setAxisSpeed(Dimensionless axisSpeed) {
    axisSpeed.times(TUNNEL.AXIS_MAX_SPEED);
    setSpeed(axisSpeed);
  }

  public void setSpeed(Dimensionless percentOutput) {
    m_motor.set(percentOutput.in(Value));
  }

  public void stop() {
    m_motor.stopMotor();
  }
}
