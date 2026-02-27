package frc.robot.commands.spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SpindexerStop extends Command {

  public SpindexerStop() {
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
