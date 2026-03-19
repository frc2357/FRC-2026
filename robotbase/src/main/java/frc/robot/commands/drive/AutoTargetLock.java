package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoTargetLock extends Command {

  public AutoTargetLock() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void execute() {
    Rotation2d target = Robot.shotCalculator.getCalculatedDriveAngle();
    Robot.swerve.driveAtAngle(
      MetersPerSecond.zero(),
      MetersPerSecond.zero(),
      target
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
