package frc.robot.subsystems.floor;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.FLOOR;

public class FloorSim {

  private final FlywheelSim m_flywheelSim;
  private final SparkMaxSim m_motorSim;

  public FloorSim(SparkMax motor) {
    m_flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        FLOOR.GEARBOX,
        FLOOR.MOI.baseUnitMagnitude(),
        FLOOR.GEARING.getMechanismToRotorRatio()
      ),
      FLOOR.GEARBOX
    );

    m_motorSim = new SparkMaxSim(motor, FLOOR.GEARBOX);
  }

  public void update() {
    m_flywheelSim.setInput(
      m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
    );

    m_flywheelSim.update(0.02);

    m_motorSim.iterate(
      m_flywheelSim.getAngularVelocityRPM() *
        FLOOR.GEARING.getMechanismToRotorRatio(),
      RoboRioSim.getVInVoltage(),
      0.02
    );

    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrentDrawAmps())
    );
  }

  public double getVelocityRPM() {
    return m_flywheelSim.getAngularVelocityRPM();
  }

  public double getCurrentDrawAmps() {
    return m_flywheelSim.getCurrentDrawAmps();
  }
}
