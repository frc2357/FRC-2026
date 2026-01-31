package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetRPM extends Command {

  private AngularVelocity m_targetRPM;

  public ShooterSetRPM(AngularVelocity targetRPM) {
    addRequirements(Robot.shooter);
    m_targetRPM = targetRPM;
  }

  @Override
  public void initialize() {
    Robot.shooter.setRPM(m_targetRPM);
  }

  @Override
  public void periodic() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
