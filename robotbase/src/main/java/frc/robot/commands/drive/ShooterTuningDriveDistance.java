package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterTuningDriveDistance extends Command {

  Distance m_targetDistanceFromHub = Inches.of(0);

  PIDController m_PidController;

  public ShooterTuningDriveDistance() {
    //Drives facing the hub to the specified distance from hub
    addRequirements(Robot.swerve);
    SmartDashboard.putNumber("ShooterTuningDriveDistance", 70);
    m_PidController = new PIDController(.1, 0, 0);
    m_PidController.setTolerance(
      Constants.SWERVE.SHOOTER_TUNING_DISTANCE_TOLERANCE.in(Inches)
    );
  }

  @Override
  public void execute() {
    Distance distanceFromHub = Robot.shotCalculator.getDistance();
    Rotation2d target = Robot.shotCalculator.getCalculatedDriveAngle();
    m_targetDistanceFromHub = Inches.of(
      SmartDashboard.getNumber("ShooterTuningDriveDistance", 0)
    );

    LinearVelocity speed = MetersPerSecond.of(
      m_PidController.calculate(
        distanceFromHub.in(Inches),
        m_targetDistanceFromHub.in(Inches)
      )
    );

    SmartDashboard.putNumber("TuningDriveSpeed", speed.in(MetersPerSecond));
    if (
      !distanceFromHub.isNear(
        m_targetDistanceFromHub,
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
