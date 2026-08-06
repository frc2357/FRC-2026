package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.HOOD;

public class Hood extends SubsystemBase {

  private HoodSim m_sim;
  private ClosedLoopSlot m_closedLoopSlot = ClosedLoopSlot.kSlot0;

  private SparkMax m_motor;
  private SparkAbsoluteEncoder m_encoder;

  private SparkClosedLoopController m_pidController;

  private MutAngle m_targetAngle = Units.Degrees.mutable(Double.NaN);
  private MutAngle m_currentAngleHolder = Units.Degrees.mutable(Double.NaN);

  public Hood() {
    m_motor = new SparkMax(CAN_ID.HOOD_MOTOR, MotorType.kBrushless);

    m_motor.configure(
      HOOD.MOTOR_CONFIG,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );

    m_pidController = m_motor.getClosedLoopController();

    m_encoder = m_motor.getAbsoluteEncoder();

    if (RobotBase.isSimulation()) {
      m_sim = new HoodSim(m_motor);
      m_closedLoopSlot = HOOD.SIM_CLOSED_LOOP_SLOT;
    }
  }

  public Angle getAngle() {
    m_currentAngleHolder.mut_replace(m_encoder.getPosition(), Units.Rotations);
    return m_currentAngleHolder;
  }

  public void setAngle(Angle angle) {
    m_targetAngle.mut_replace(angle);
    System.out.println(
      "Hood.setAngle() - m_targetAngle: " +
        m_targetAngle.in(Degrees) +
        " degrees | m_encoder.getPosition(): " +
        m_encoder.getPosition() +
        " rotations | m_encoder.getPosition() in degrees: " +
        m_encoder.getPosition() * 360 +
        " degrees | m_motor.getAppliedOutput(): " +
        m_motor.getAppliedOutput() +
        " | m_pidController.isAtSetpoint(): " +
        m_pidController.isAtSetpoint()
    );
    m_pidController.setSetpoint(
      m_targetAngle.in(Degrees),
      ControlType.kPosition,
      m_closedLoopSlot
    );
  }

  public void goHome() {
    setAngle(HOOD.SETPOINTS.HOME);
  }

  public void set(double dutyCycle) {
    m_motor.set(dutyCycle);
  }

  public void set(Dimensionless speed) {
    set(speed.in(Value));
  }

  public void axisSpeed(Dimensionless speed) {
    set(speed.times(HOOD.AXIS_MAX_SPEED));
  }

  public void stop() {
    m_motor.stopMotor();
  }

  @Override
  public void simulationPeriodic() {
    m_sim.update();

    SmartDashboard.putNumber(
      "New Hood Motor Applied Output",
      m_motor.getAppliedOutput()
    );
    SmartDashboard.putNumber(
      "New Hood Motor Velocity (RPM)",
      m_encoder.getVelocity()
    );
    SmartDashboard.putNumber(
      "New Hood Angle (Degrees)",
      getAngle().in(Degrees)
    );
    SmartDashboard.putNumber("New Hood Angle Raw", m_encoder.getPosition());
    SmartDashboard.putData("New Hood Subsystem", this);
  }
}
