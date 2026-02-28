package frc.robot.commands.floor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FloorStop extends Command {

  public FloorStop() {
    addRequirements(Robot.floor);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.floor.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
