package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ResetDriveModifiers extends Command {

  public ResetDriveModifiers() {}

  @Override
  public void initialize() {
    Robot.swerve.resetDriveModifiers();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
