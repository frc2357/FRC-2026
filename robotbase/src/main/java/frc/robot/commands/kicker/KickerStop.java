package frc.robot.commands.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class KickerStop extends Command {

  public KickerStop() {
    addRequirements(Robot.kicker);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.kicker.stop();
  }
}
