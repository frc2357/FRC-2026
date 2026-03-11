package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ToggleDefaultHood extends Command {

  public ToggleDefaultHood() {
    addRequirements(Robot.hood);
  }

  @Override
  public void initialize() {
    if (Robot.hood.getDefaultCommand() != null) {
      Robot.hood.removeDefaultCommand();
      System.out.println("----- DISABLED HOOD DEFAULT COMMAND ----");
    } else {
      Robot.hood.setDefaultCommand(Robot.hood.goHome());
      System.out.println("---- ENABLED HOOD DEFAULT COMMAND ----");
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
