package frc.robot.commands.hood;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class HoodSetSpeed extends Command {

  private Dimensionless m_speed;

  public HoodSetSpeed(Dimensionless speed) {
    m_speed = speed;
    addRequirements(Robot.hood);
  }

  @Override
  public void initialize() {
    Robot.hood.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.hood.stop();
  }
}
