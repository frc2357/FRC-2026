package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.INTAKE_PIVOT;

public class IntakePivotSim {

  private final SingleJointedArmSim m_armSim;
  private final SparkMaxSim m_motorSim;
  private final SparkAbsoluteEncoderSim m_absoluteEncoderSim;

  public IntakePivotSim(SparkMax motor) {
    m_armSim = new SingleJointedArmSim(
      LinearSystemId.createSingleJointedArmSystem(
        INTAKE_PIVOT.GEARBOX,
        INTAKE_PIVOT.MOI.in(KilogramSquareMeters),
        INTAKE_PIVOT.GEARING.getRotorToMechanismRatio()
      ),
      INTAKE_PIVOT.GEARBOX,
      INTAKE_PIVOT.GEARING.getRotorToMechanismRatio(),
      INTAKE_PIVOT.LENGTH.baseUnitMagnitude(),
      INTAKE_PIVOT.SIM_LOWER_ANGLE.in(Radians),
      INTAKE_PIVOT.SIM_UPPER_ANGLE.in(Radians),
      false, // simulate gravity
      INTAKE_PIVOT.SIM_STARTING_POSITION.in(Radians)
    );

    m_motorSim = new SparkMaxSim(motor, INTAKE_PIVOT.GEARBOX);
    m_absoluteEncoderSim = m_motorSim.getAbsoluteEncoderSim();
  }

  public void update() {
    m_armSim.setInput(
      m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
    );

    m_armSim.update(0.02);

    m_motorSim.iterate(
      RadiansPerSecond.of(m_armSim.getVelocityRadPerSec()).in(RPM) *
        INTAKE_PIVOT.GEARING.getRotorToMechanismRatio(),
      RoboRioSim.getVInVoltage(),
      0.02
    );

    m_absoluteEncoderSim.setPosition(
      Units.radiansToRotations(m_armSim.getAngleRads())
    );
    m_absoluteEncoderSim.setVelocity(
      Units.radiansToRotations(m_armSim.getVelocityRadPerSec())
    );

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrentDrawAmps())
    );
  }

  public double getAngleRads() {
    return m_armSim.getAngleRads();
  }

  public double getCurrentDrawAmps() {
    return m_armSim.getCurrentDrawAmps();
  }
}
