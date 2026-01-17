package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {

    private SparkMax m_motor;

    public Spindexer() {
        m_motor = new SparkMax(CAN_ID.SPINDEXER_MOTOR, MotorType.kBrushless);
        m_motor.configure(
                SPINDEXER.MOTOR_CONFIG,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }
}
