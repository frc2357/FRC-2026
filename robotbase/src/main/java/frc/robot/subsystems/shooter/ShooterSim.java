package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.SHOOTER;

public class ShooterSim {

  private final FlywheelSim m_flywheelSim;
  private final SparkMaxSim m_leaderMotorSim;
  private final SparkMaxSim m_followerMotorSim;

  public ShooterSim(SparkMax leaderMotor, SparkMax followerMotor) {
    m_flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        SHOOTER.GEARBOX,
        SHOOTER.MOI.baseUnitMagnitude(),
        SHOOTER.GEARING.getMechanismToRotorRatio()
      ),
      SHOOTER.GEARBOX
    );

    m_leaderMotorSim = new SparkMaxSim(leaderMotor, SHOOTER.GEARBOX);
    m_followerMotorSim = new SparkMaxSim(followerMotor, SHOOTER.GEARBOX);
  }

  public void update() {
    m_flywheelSim.setInput(
      m_leaderMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage() +
        m_followerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage()
    );

    m_flywheelSim.update(0.02);

    m_leaderMotorSim.iterate(
      m_flywheelSim.getAngularVelocityRPM() *
        SHOOTER.GEARING.getMechanismToRotorRatio(),
      RoboRioSim.getVInVoltage(),
      0.02
    );

    m_followerMotorSim.iterate(
      m_flywheelSim.getAngularVelocityRPM() *
        SHOOTER.GEARING.getMechanismToRotorRatio(),
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
