package frc.robot.commands.floor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FloorStop extends Command {

  public FloorStop() {
    addRequirements(Robot.floor);
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean interrupted) {
    Robot.floor.stop();
  }
}
