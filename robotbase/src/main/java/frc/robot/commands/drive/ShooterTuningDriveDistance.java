package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterTuningDriveDistance extends Command {

  Distance m_targetDistanceFromHub = Inches.of(0);

  public ShooterTuningDriveDistance() {
    //Drives facing the hub to the specified distance from hub
    addRequirements(Robot.swerve);
    SmartDashboard.putNumber("ShooterTuningDriveDistance", 70);
  }

  @Override
  public void execute() {
    Distance distanceFromHub = Robot.shotCalculator.getDistance();
    Rotation2d target = Robot.shotCalculator.getCalculatedDriveAngle();
    m_targetDistanceFromHub = Inches.of(
      SmartDashboard.getNumber("ShooterTuningDriveDistance", 0)
    );

    if (
      distanceFromHub.lt(
        m_targetDistanceFromHub.minus(
          Constants.SWERVE.SHOOTER_TUNING_DISTANCE_TOLERANCE
        )
      )
    ) {
      Robot.swerve.driveAtAngle(
        Constants.SWERVE.SHOOTER_TUNING_DRIVE_SPEED.times(-target.getCos()),
        Constants.SWERVE.SHOOTER_TUNING_DRIVE_SPEED.times(-target.getSin()),
        target
      );
    }

    if (
      distanceFromHub.gt(
        m_targetDistanceFromHub.plus(
          Constants.SWERVE.SHOOTER_TUNING_DISTANCE_TOLERANCE
        )
      )
    ) {
      Robot.swerve.driveAtAngle(
        Constants.SWERVE.SHOOTER_TUNING_DRIVE_SPEED.times(target.getCos()),
        Constants.SWERVE.SHOOTER_TUNING_DRIVE_SPEED.times(target.getSin()),
        target
      );
    } else {
      Robot.swerve.driveAtAngle(
        MetersPerSecond.zero(),
        MetersPerSecond.zero(),
        target
      );
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
