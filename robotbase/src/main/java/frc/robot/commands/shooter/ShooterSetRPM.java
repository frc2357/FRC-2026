package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetRPM extends Command {

  private AngularVelocity m_targetVelocity;

  public ShooterSetRPM(AngularVelocity targetVelocity) {
    addRequirements(Robot.shooter);
    m_targetVelocity = targetVelocity;
  }

  @Override
  public void initialize() {
    Robot.shooter.setTargetVelocity(m_targetVelocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
