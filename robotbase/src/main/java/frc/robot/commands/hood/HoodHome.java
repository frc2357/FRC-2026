package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class HoodHome extends Command {

  public HoodHome() {
    addRequirements(Robot.hood);
  }

  @Override
  public void initialize() {
    Robot.hood.goHome();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
