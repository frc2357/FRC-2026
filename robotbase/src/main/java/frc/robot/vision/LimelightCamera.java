package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.LIMELIGHT;
import frc.robot.Robot;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import org.photonvision.EstimatedRobotPose;

public class LimelightCamera implements CameraInterface {

  private Limelight m_camera;

  protected Optional<PoseEstimate> m_poseEstimate = Optional.empty();

  private Matrix<N3, N1> m_multiTagStdDevs;

  private Matrix<N3, N1> m_currentStdDevs;

  public LimelightCamera(
    String cameraName,
    Pose3d robotToCameraTransform,
    Matrix<N3, N1> multiTagStdDevs
  ) {
    m_camera = new Limelight(cameraName);
    m_camera
      .getSettings()
      .withLimelightLEDMode(LEDMode.PipelineControl)
      .withCameraOffset(robotToCameraTransform)
      .withPipelineIndex(VisionPipeline.MULTI_TAG_PIPELINE.getIndex())
      .withImuMode(ImuMode.SyncInternalImu)
      .save();

    m_multiTagStdDevs = multiTagStdDevs;
    m_currentStdDevs = multiTagStdDevs;

    RobotModeTriggers.disabled().onTrue(
      Commands.run(() -> {
        SmartDashboard.putBoolean("ll dis", true);
        SmartDashboard.putBoolean("ll en", false);

        m_camera
          .getSettings()
          .withThrottle(LIMELIGHT.DISABLED_THERMAL_THROTTLE)
          .withImuMode(ImuMode.SyncInternalImu)
          .save();
      }).ignoringDisable(true)
    );
    RobotModeTriggers.disabled().onFalse(
      Commands.run(() -> {
        SmartDashboard.putBoolean("ll en", true);
        SmartDashboard.putBoolean("ll dis", false);

        m_camera
          .getSettings()
          .withThrottle(LIMELIGHT.ENABLED_THERMAL_THROTTLE)
          .withImuMode(ImuMode.InternalImuExternalAssist)
          .save();
      })
    );
  }

  @Override
  public void updateResult() {
    m_camera
      .getSettings()
      .withRobotOrientation(Robot.swerve.getFieldRelativeOrientation3d())
      .save();

    Optional<PoseEstimate> visionEstimate = BotPose.BLUE_MEGATAG2.get(m_camera);
    // filter here
    m_poseEstimate = visionEstimate;
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
        m_currentStdDevs,
        est.timestampSeconds
      )
    );
  }

  @Override
  public void setPipeline(VisionPipeline pipeline) {
    m_camera.getSettings().withPipelineIndex(pipeline.getIndex()).save();
  }
}
