package frc.robot.subsystems.intakerunner;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.INTAKE_RUNNER;

public class IntakeRunnerSim {

  private final FlywheelSim m_flywheelSim;
  private final TalonFXSimState m_leaderMotorSim;
  private final TalonFXSimState m_followerMotorSim;

  public IntakeRunnerSim(TalonFX leaderMotor, TalonFX followerMotor) {
    m_flywheelSim = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(
        INTAKE_RUNNER.GEARBOX,
        INTAKE_RUNNER.MOI.baseUnitMagnitude(),
        INTAKE_RUNNER.GEARING.getMechanismToRotorRatio()
      ),
      INTAKE_RUNNER.GEARBOX
    );

    m_leaderMotorSim = leaderMotor.getSimState();
    m_followerMotorSim = followerMotor.getSimState();
  }

  public void update() {
    m_leaderMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
    m_followerMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

    m_flywheelSim.setInputVoltage(m_leaderMotorSim.getMotorVoltage());
    m_flywheelSim.update(0.02);

    double rotorRPS =
      (m_flywheelSim.getAngularVelocityRPM() / 60.0) *
      INTAKE_RUNNER.GEARING.getMechanismToRotorRatio();

    m_leaderMotorSim.setRotorVelocity(rotorRPS);
    m_leaderMotorSim.addRotorPosition(rotorRPS * 0.02);
    m_followerMotorSim.setRotorVelocity(rotorRPS);
    m_followerMotorSim.addRotorPosition(rotorRPS * 0.02);

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
