package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveTargetingHub extends Command {

  Supplier<Dimensionless> m_x;
  Supplier<Dimensionless> m_y;
  Supplier<Dimensionless> m_rotation;

  public DriveTargetingHub(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y,
    Supplier<Dimensionless> rotation
  ) {
    addRequirements(Robot.swerve);
    m_x = x;
    m_y = y;
    m_rotation = rotation;

    SmartDashboard.putNumber("Target Rotation", 0);
  }

  @Override
  public void initialize() {
    Robot.backLeftCam.setPipeline(0); // April tag pipeline TODO: make a constant
    SmartDashboard.putBoolean("computing", false);
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

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("computing", false);
  }

  public Optional<Rotation2d> computeTargetAngle() {
    double rawYaw = Robot.backLeftCam.getNewestTargetYaw(50);
    if (Double.isNaN(rawYaw)) {
      SmartDashboard.putBoolean("computing", false);
      return Optional.empty();
    }

    Rotation2d cameraYaw = Rotation2d.fromDegrees(
      Robot.backLeftCam.getNewestTargetYaw(50)
    );

    Rotation2d robotHeading = Robot.swerve.getRobotRotation();

    // Convert camera yaw into field-relative bearing to the AprilTag
    Rotation2d fieldBearing = robotHeading.plus(cameraYaw); // -90
    // This is the direction, in field coordinates, from the robot toward the AprilTag

    Translation2d targetHub = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );
    Pose3d aprilTag = FieldConstants.FIELD_LAYOUT.getTagPose(21).get(); // TODO: FIX HARDCODED ID

    Rotation2d fieldBearingFiducialToHub = Rotation2d.fromRadians(
      Math.atan2(
        targetHub.getY() - aprilTag.getY(),
        targetHub.getX() - aprilTag.getX()
      )
    );

    Rotation2d angle_offset = fieldBearingFiducialToHub
      .minus(fieldBearing)
      .div(2);
    Rotation2d target_angle = robotHeading.plus(angle_offset);
    return Optional.of(target_angle);
  }
}
