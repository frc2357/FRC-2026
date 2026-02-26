package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.Optional;
import java.util.function.Supplier;

public class DriveNaiveTargetingHub extends Command {

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

  // This is the tag on the right side of the blue hub when looking from the field origin
  private static final int TARGET_ID = 21;

  public DriveNaiveTargetingHub(
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
  public void execute() {
    Optional<Rotation2d> target = computeTargetAngle();
    if (target.isEmpty()) {
      Robot.swerve.driveFieldRelative(
        m_y
          .get()
          .times(Constants.SWERVE.AXIS_MAX_SPEED)
          .times(SWERVE.MAX_SPEED),
        m_x
          .get()
          .times(Constants.SWERVE.AXIS_MAX_SPEED)
          .times(SWERVE.MAX_SPEED),
        m_rotation
          .get()
          .times(Constants.SWERVE.AXIS_MAX_ANGULAR_RATE)
          .times(SWERVE.MAX_ANGULAR_RATE)
      );
    } else {
      Robot.swerve.driveAtAngle(
        m_y
          .get()
          .times(Constants.SWERVE.AXIS_MAX_SPEED)
          .times(SWERVE.MAX_SPEED),
        m_x
          .get()
          .times(Constants.SWERVE.AXIS_MAX_SPEED)
          .times(SWERVE.MAX_SPEED),
        target.get()
      );
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public Optional<Rotation2d> computeTargetAngle() {
    double rawYaw = Robot.cameraManager.m_shooter.getTargetYaw(
      DriveNaiveTargetingHub.TARGET_ID,
      PHOTON_VISION.NAIVE_APRIL_TAG_TARGET_TIMEOUT
    );
    if (Double.isNaN(rawYaw)) {
      return Optional.empty();
    }

    Rotation2d cameraYaw = Rotation2d.fromDegrees(rawYaw);

    Rotation2d robotHeading = Robot.swerve
      .getFieldRelativePose2d()
      .getRotation();

    // Convert camera yaw into field-relative bearing to the AprilTag
    Rotation2d fieldBearing = robotHeading.plus(cameraYaw);
    // This is the direction, in field coordinates, from the robot toward the AprilTag

    Translation2d targetHub = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );
    Pose3d aprilTag = FieldConstants.FIELD_LAYOUT.getTagPose(
      DriveNaiveTargetingHub.TARGET_ID
    ).get();

    Rotation2d fieldBearingFiducialToHub = Rotation2d.fromRadians(
      Math.atan2(
        targetHub.getY() - aprilTag.getY(),
        targetHub.getX() - aprilTag.getX()
      )
    );

    Rotation2d angle_offset = fieldBearingFiducialToHub.minus(fieldBearing);
    Rotation2d target_angle = robotHeading.plus(angle_offset);
    return Optional.of(target_angle);
  }
}
