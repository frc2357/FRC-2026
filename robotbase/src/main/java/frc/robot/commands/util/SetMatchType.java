package frc.robot.commands.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetMatchType extends Command {

  @Override
  public boolean isFinished() {
    return DriverStation.isDSAttached();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.matchType = DriverStation.getMatchType();
    Robot.initializeTuningController();
  }
}
