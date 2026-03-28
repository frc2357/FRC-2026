package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ToggleDefaultShooter extends Command {

  public ToggleDefaultShooter() {
    addRequirements(Robot.shooter);
  }

  @Override
  public void initialize() {
    if (Robot.hood.getDefaultCommand() != null) {
      Robot.shooter.removeDefaultCommand();
      System.out.println("----- DISABLED SHOOTER DEFAULT COMMAND ----");
    } else {
      Robot.shooter.setDefaultCommand(Robot.shooter.setIdleVelocity());
      System.out.println("---- ENABLED SHOOTER DEFAULT COMMAND ----");
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
