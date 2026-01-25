package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterSetSpeed extends Command {

  private Dimensionless m_speed;

  public ShooterSetSpeed(Dimensionless speed) {
    m_speed = speed;
    addRequirements(Robot.shooter);
  }

  @Override
  public void initialize() {
    Robot.shooter.setSpeed(m_speed);
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
