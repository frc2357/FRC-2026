package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.HOOD;

public class HoodSim {

  private final SingleJointedArmSim m_armSim;
  private final SparkMaxSim m_motorSim;
  private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;

  public HoodSim(SparkMax motor) {
    m_armSim = new SingleJointedArmSim(
      LinearSystemId.createSingleJointedArmSystem(
        HOOD.GEARBOX,
        SingleJointedArmSim.estimateMOI(
          HOOD.LENGTH.baseUnitMagnitude(),
          HOOD.MASS.baseUnitMagnitude()
        ),
        HOOD.GEARING.getMechanismToRotorRatio()
      ),
      HOOD.GEARBOX,
      HOOD.GEARING.getMechanismToRotorRatio(),
      HOOD.LENGTH.baseUnitMagnitude(),
      HOOD.LOWER_ANGLE_LIMIT.in(Radians),
      HOOD.UPPER_ANGLE_LIMIT.in(Radians),
      false, // simulate gravity
      HOOD.SIM_STARTING_POSITION.in(Radians)
    );

    m_motorSim = new SparkMaxSim(motor, HOOD.GEARBOX);
    m_absoluteEncoderSim = m_motorSim.getAbsoluteEncoderSim();
  }

  public void update() {
    SmartDashboard.putNumber(
      "Hood applied output",
      m_motorSim.getAppliedOutput()
    );
    m_armSim.setInput(
      m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
    );

    m_armSim.update(0.02);

    m_motorSim.iterate(
      ((m_armSim.getVelocityRadPerSec() * 60.0) / (2 * Math.PI)) *
        HOOD.GEARING.getMechanismToRotorRatio(), // convert to motor RPM
      RoboRioSim.getVInVoltage(),
      0.02
    );

    // 3. Write simulated position and velocity back to the absolute encoder sim.
    //    The Hood subsystem reads position/velocity from the absolute encoder,
    //    not the built-in relative encoder, so we update it directly here.
    //    Both values are divided by the encoder gearing ratio to convert from
    //    mechanism space (what the arm sim outputs) to encoder-shaft space
    //    (what the Spark MAX absolute encoder expects to read).
    m_absoluteEncoderSim.setPosition(
      Units.radiansToRotations(m_armSim.getAngleRads())
    );
    m_absoluteEncoderSim.setVelocity(
      Units.radiansToRotations(m_armSim.getVelocityRadPerSec())
    );

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrentDrawAmps())
    );
    SmartDashboard.putNumber(
      "Hood angle (degrees)",
      Units.radiansToDegrees(m_armSim.getAngleRads())
    );

    // System.out.println(
    //   "HoodSim.update() - m_armSim.getAngleRads(): " +
    //     m_armSim.getAngleRads() +
    //     " radians | m_absoluteEncoderSim.getPosition(): " +
    //     m_absoluteEncoderSim.getPosition() +
    //     " rotations | m_absoluteEncoderSim.getPosition() in degrees: " +
    //     Rotations.of(m_absoluteEncoderSim.getPosition()).in(Degrees) +
    //     " degrees | m_motorSim.getAppliedOutput(): " +
    //     m_motorSim.getAppliedOutput()
    // );
  }

  public double getAngleRads() {
    return m_armSim.getAngleRads();
  }

  public double getVelocityRadPerSec() {
    return m_armSim.getVelocityRadPerSec();
  }

  public double getCurrentDrawAmps() {
    return m_armSim.getCurrentDrawAmps();
  }
}
