package frc.robot.commands.spindexer;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpindexerSetSpeed extends Command {

  private Dimensionless m_speed;

  public SpindexerSetSpeed(Dimensionless speed) {
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
    Robot.floor.stop();
  }
}
