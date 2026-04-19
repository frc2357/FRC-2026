package frc.robot.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants;
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
  private Matrix<N3, N1> m_currentStdDevs;

  private Field2d m_field = new Field2d();

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
      .withThrottle(LIMELIGHT.ENABLED_THERMAL_THROTTLE)
      .withImuMode(ImuMode.SyncInternalImu)
      .save();

    m_tagStdDevs = tagStdDevs;

    m_currentStdDevs = VecBuilder.fill(
      Double.MAX_VALUE,
      Double.MAX_VALUE,
      Double.MAX_VALUE
    );

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
      .or(RobotModeTriggers.autonomous())
      .onTrue(
        Commands.run(() -> {
          m_camera
            .getSettings()
            .withThrottle(LIMELIGHT.ENABLED_THERMAL_THROTTLE)
            .withImuMode(ImuMode.InternalImuExternalAssist)
            .save();
        })
      );
    SmartDashboard.putData(cameraName + " field", m_field);
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
    m_currentStdDevs = m_tagStdDevs;
    // Uncomment this if we notice a lot of jitter at long distances
    //m_currentStdDevs = updateEstimationStdDevs(m_poseEstimate.get());
    m_field.setRobotPose(m_poseEstimate.get().pose.toPose2d());
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

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * Based on photonvision example code: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> updateEstimationStdDevs(PoseEstimate estimatedPose) {
    // Pose present. Start running Heuristic
    var estStdDevs = m_tagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (var tgt : estimatedPose.rawFiducials) {
      var tagPose = Constants.FieldConstants.FIELD_LAYOUT.getTagPose(tgt.id);
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist += tagPose
        .get()
        .toPose2d()
        .getTranslation()
        .getDistance(estimatedPose.pose.toPose2d().getTranslation());
    }

    // One or more tags visible, run the full heuristic.
    avgDist /= numTags;
    // Increase std devs based on (average) distance
    if (avgDist > 4) estStdDevs = VecBuilder.fill(
      Double.MAX_VALUE,
      Double.MAX_VALUE,
      Double.MAX_VALUE
    );
    else estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30));
    return estStdDevs;
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
