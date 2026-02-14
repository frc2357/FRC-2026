package frc.robot.commands.intakepivot;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakePivotSetSpeed extends Command {

  private Dimensionless m_speed;

  public IntakePivotSetSpeed(Dimensionless speed) {
    m_speed = speed;
    addRequirements(Robot.intakePivot);
  }

  @Override
  public void initialize() {
    Robot.intakePivot.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakePivot.stop();
  }
}
