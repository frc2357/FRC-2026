package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CrossWheels extends Command {

  public CrossWheels() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    Robot.swerve.brakeX();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
