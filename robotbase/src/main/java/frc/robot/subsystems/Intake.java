import frc.robot.Constants.CAN_ID;

public class Intake extends SubsystemBase {

    private SparkMax m_motor;

    public Intake() {
        m_motor = new SparkMax(CAN_ID.INTAKE_MOTOR, MotorType.kBrushless);

        m_motor.configure(
            INTAKE_MOTOR.MOTOR_CONFIG,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public void setSpeed(double percentOutput) {
        m_motor.set(percentOutput);
    }

    public void setAxisSpeed(double axisSpeed) {
        axis Speed *= INTAKE.AXIS_MAX_SPEED;
        setSpeed(axisSpeed);
    }

    public void stop() {
        m_motor.stopMotor();
    }
}