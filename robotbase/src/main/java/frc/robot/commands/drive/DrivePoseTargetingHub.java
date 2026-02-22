package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;

public class DrivePoseTargetingHub extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;

  public DrivePoseTargetingHub(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;
  }

  @Override
  public void initialize() {
    Robot.cameraManager.setPipeline(PHOTON_VISION.MULTI_TAG_PIPELINE);
  }

  @Override
  public void execute() {
    Rotation2d target = computeTargetAngle();
    SmartDashboard.putNumber("Target angle", target.getDegrees());
    Robot.swerve.driveAtAngle(
      m_y.get().times(Constants.SWERVE.AXIS_MAX_SPEED).times(SWERVE.MAX_SPEED),
      m_x.get().times(Constants.SWERVE.AXIS_MAX_SPEED).times(SWERVE.MAX_SPEED),
      target
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static Rotation2d computeTargetAngle() {
    Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();
    Translation2d target = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );
    // Start with the angle from robot center to target
    Rotation2d driveAngleGuess = target
      .minus(robotPose.getTranslation())
      .getAngle();

    // Run 2 iterations to account for the launcher 'swinging' as the robot rotates
    for (int i = 0; i < 2; i++) {
      // 1. Find where the launcher would be on the field if the robot faces our guess
      Translation2d projectedLauncherPose = robotPose
        .getTranslation()
        .plus(
          Constants.SHOOTER.ROBOT_TO_SHOOTER.getTranslation().rotateBy(
            driveAngleGuess
          )
        );

      // 2. Find the angle from that projected launcher position to the target
      Rotation2d launcherToTargetAngle = target
        .minus(projectedLauncherPose)
        .getAngle();

      // 3. Subtract the launcher's mount angle from the required pointing angle
      // If launcher is at -50, robot must be at (TargetAngle - (-50))
      driveAngleGuess = launcherToTargetAngle.minus(
        Constants.SHOOTER.ROBOT_TO_SHOOTER.getRotation()
      );
    }

    return driveAngleGuess;
  }
}
