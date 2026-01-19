package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ResetPerspective extends Command {

  public ResetPerspective() {
    addRequirements(Robot.swerve);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.seedFieldCentric();
  }
}
