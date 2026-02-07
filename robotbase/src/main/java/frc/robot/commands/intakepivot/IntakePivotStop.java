package frc.robot.commands.intakePivot;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakePivotStop extends Command {

  public IntakePivotStop() {
    addRequirements(Robot.intakePivot);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakePivot.stop();
  }
}
