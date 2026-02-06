package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakePivotStop extends Command {

  public IntakePivotStop() {
    addRequirements(Robot.intakepivot);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakepivot.stop();
  }
}
