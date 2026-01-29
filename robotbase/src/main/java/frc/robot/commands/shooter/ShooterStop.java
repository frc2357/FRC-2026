package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShooterStop extends Command {

  public ShooterStop() {
    addRequirements(Robot.shooter);
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean interrupted) {
    Robot.shooter.stop();
  }
}
