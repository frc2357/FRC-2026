package frc.robot.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.Robot;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class LimelightCamera implements CameraInterface {

  private Limelight m_camera;

  protected Optional<PoseEstimate> m_poseEstimate = Optional.empty();

  private Matrix<N3, N1> m_tagStdDevs;

  private Field2d field = new Field2d();

  public LimelightCamera(
    String cameraName,
    Pose3d robotToCameraTransform,
    Matrix<N3, N1> tagStdDevs
  ) {
    m_camera = new Limelight(cameraName);
    m_camera
      .getSettings()
      .withLimelightLEDMode(LEDMode.PipelineControl)
      .withCameraOffset(robotToCameraTransform)
      .withPipelineIndex(VisionPipeline.MULTI_TAG_PIPELINE.getIndex())
      .withThrottle(LIMELIGHT.DISABLED_THERMAL_THROTTLE)
      .withImuMode(ImuMode.SyncInternalImu)
      .save();

    m_tagStdDevs = tagStdDevs;

    RobotModeTriggers.disabled().onTrue(
      Commands.run(() -> {
        m_camera
          .getSettings()
          .withThrottle(LIMELIGHT.DISABLED_THERMAL_THROTTLE)
          .withImuMode(ImuMode.SyncInternalImu)
          .save();
      }).ignoringDisable(true)
    );
    RobotModeTriggers.teleop()
      .or(RobotModeTriggers.test())
      .onTrue(
        Commands.run(() -> {
          m_camera
            .getSettings()
            .withThrottle(LIMELIGHT.ENABLED_THERMAL_THROTTLE)
            .withImuMode(ImuMode.InternalImuExternalAssist)
            .save();
        })
      );

    SmartDashboard.putData(cameraName + " field", field);
    SmartDashboard.putBoolean("seeded", false);
  }

  public void seedGyroMethod(Orientation3d robotOrientation) {
    SmartDashboard.putBoolean("seeded", true);
    m_camera
      .getSettings()
      .withImuMode(ImuMode.SyncInternalImu)
      .withRobotOrientation(robotOrientation)
      .save();
  }

  public void useInternalImu() {
    m_camera
      .getSettings()
      .withImuMode(ImuMode.InternalImuExternalAssist)
      .save();
  }

  @Override
  public void updateResult() {
    m_camera
      .getSettings()
      .withRobotOrientation(Robot.swerve.getFieldRelativeOrientation3d())
      .save();
    m_poseEstimate = Optional.empty();

    Optional<PoseEstimate> visionEstimate = BotPose.BLUE_MEGATAG2.get(m_camera);

    if (visionEstimate.isEmpty()) {
      return;
    }

    if (visionEstimate.get().tagCount == 0) {
      return;
    }

    if (!passesRobotSpeedFilter(visionEstimate.get())) {
      return;
    }

    // filter here
    m_poseEstimate = visionEstimate;

    field.setRobotPose(m_poseEstimate.get().pose.toPose2d());
  }

  /**
   *
   * @param estimate The camera's pose estimate
   * @return true if the vision estimate is within reasonable constraints to the robot
   */
  public boolean passesRobotSpeedFilter(PoseEstimate estimate) {
    ChassisSpeeds speeds = Robot.swerve.getCurrentRobotRelativeSpeeds();

    // Check if we are rotating too fast
    if (
      speeds.omegaRadiansPerSecond >=
      LIMELIGHT.MAX_ROBOT_ROTATION.in(RadiansPerSecond)
    ) {
      return false;
    }

    return true;
  }

  @Override
  public Optional<SwervePoseEstimate> getEstimateForSwerve() {
    if (m_poseEstimate.isEmpty()) {
      return Optional.empty();
    }
    var est = m_poseEstimate.get();
    return Optional.of(
      new SwervePoseEstimate(
        est.pose.toPose2d(),
        m_tagStdDevs,
        est.timestampSeconds
      )
    );
  }

  @Override
  public void setPipeline(VisionPipeline pipeline) {
    m_camera.getSettings().withPipelineIndex(pipeline.getIndex()).save();
  }
}
