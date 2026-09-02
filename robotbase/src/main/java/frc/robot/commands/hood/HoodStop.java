package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class HoodStop extends Command {

  public HoodStop() {
    addRequirements(Robot.hood);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.hood.stop();
  }
}
