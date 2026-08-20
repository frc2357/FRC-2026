package frc.robot.subsystems.feeder;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.FEEDER;

public class FeederSim {

  private final FlywheelSim m_flywheelSim;
  private final SparkMaxSim m_motorSim;

  public FeederSim(SparkMax motor) {
    m_flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        FEEDER.GEARBOX,
        FEEDER.MOI.baseUnitMagnitude(),
        FEEDER.GEARING.getMechanismToRotorRatio()
      ),
      FEEDER.GEARBOX
    );

    m_motorSim = new SparkMaxSim(motor, FEEDER.GEARBOX);
  }

  public void update() {
    m_flywheelSim.setInput(
      m_motorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
    );

    m_flywheelSim.update(0.02);

    m_motorSim.iterate(
      m_flywheelSim.getAngularVelocityRPM() *
        FEEDER.GEARING.getMechanismToRotorRatio(),
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
