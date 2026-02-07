package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FeederStop extends Command {

  public FeederStop() {
    addRequirements(Robot.feeder);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.feeder.stop();
  }
}
