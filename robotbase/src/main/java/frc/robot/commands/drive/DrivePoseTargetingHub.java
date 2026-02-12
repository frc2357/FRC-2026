package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;

public class DrivePoseTargetingHub extends Command {

  /**
   * Implements a naive hub targeting command for the swerve
   *
   * This works at close-range, and along the y-axis when directly
   * facing a tag but fails at longer ranges as this strategy
   * does not account well for changes along the x-axis
   *
   * Improving this strategy directly involves calculating the robot's position, at which point we should
   * just utilize pose estimation
   *
   * This strategy could be improved upon by selecting the tag with the yaw value closest to zero to compute
   * the target angle. That will provide a wider range of "good" targeting but there will still be zones the
   * computed target angle is off.
   */

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;
  Supplier<Dimensionless> m_rotation;

  public DrivePoseTargetingHub(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y,
    Supplier<Dimensionless> rotation
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;
    m_rotation = rotation;
  }

  @Override
  public void initialize() {
    Robot.backLeftCam.setPipeline(PHOTON_VISION.NAIVE_APRIL_TAG_PIPELINE);
  }

  @Override
  public void execute() {
    Rotation2d target = computeTargetAngle();
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

  public Rotation2d computeTargetAngle() {
    Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();

    Translation2d targetHub = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );

    Rotation2d angleToHub = Rotation2d.fromRadians(
      Math.atan2(
        targetHub.getY() - robotPose.getY(),
        targetHub.getX() - robotPose.getX()
      )
    );

    return angleToHub;
  }
}
