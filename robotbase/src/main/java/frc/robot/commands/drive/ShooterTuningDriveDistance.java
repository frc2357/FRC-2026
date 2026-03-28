package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.Supplier;

public class ShooterTuningDriveDistance extends Command {

  Supplier<Distance> m_targetDistanceFromHub;

  PIDController m_PidController;

  public ShooterTuningDriveDistance(Supplier<Distance> targetDistanceFromHub) {
    //Drives facing the hub to the specified distance from hub
    addRequirements(Robot.swerve);
    m_targetDistanceFromHub = targetDistanceFromHub;
    m_PidController = new PIDController(Constants.SWERVE.SHOOTER_TUNING_DRIVE_P,Constants.SWERVE.SHOOTER_TUNING_DRIVE_I,Constants.SWERVE.SHOOTER_TUNING_DRIVE_D); 
    m_PidController.setTolerance(
      Constants.SWERVE.SHOOTER_TUNING_DISTANCE_TOLERANCE.in(Inches)
    );
  }

  @Override
  public void execute() {
    Distance distanceFromHub = Robot.shotCalculator.getDistance();
    Rotation2d target = Robot.shotCalculator.getCalculatedDriveAngle();

    LinearVelocity speed = MetersPerSecond.of(
      m_PidController.calculate(
        distanceFromHub.in(Inches),
        m_targetDistanceFromHub.get().in(Inches)
      )
    );

    SmartDashboard.putNumber("TuningDriveSpeed", speed.in(MetersPerSecond));

    if (
      !distanceFromHub.isNear(
        m_targetDistanceFromHub.get(),
        Constants.SWERVE.SHOOTER_TUNING_DISTANCE_TOLERANCE
      )
    ) {
      Robot.swerve.driveAtAngle(
        speed.times(target.getSin()),
        speed.times(-target.getCos()),
        target
      );
    } else {
      Robot.swerve.driveAtAngle(
        MetersPerSecond.of(0),
        MetersPerSecond.of(0),
        target
      );
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
