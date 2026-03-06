package frc.robot.commands.tunnel;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TunnelSetSpeed extends Command {

  private Dimensionless m_speed;

  public TunnelSetSpeed(Dimensionless speed) {
    addRequirements(Robot.floor);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.floor.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.tunnel.stop();
  }
}
