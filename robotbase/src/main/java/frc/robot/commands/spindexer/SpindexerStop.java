package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpindexerStop extends Command {

  public SpindexerStop() {
    addRequirements(Robot.spindexer);
  }

  public boolean isFinished() {
    return false;
  }

  public void end(boolean interrupted) {
    Robot.spindexer.stop();
  }
}
