package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveStop extends Command {

  public DriveStop() {
    addRequirements(Robot.swerve);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
