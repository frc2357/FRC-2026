package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.PHOTON_VISION;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Controls the photon vision camera options. */
public class PhotonVisionCamera {

  /*
   * The class for the object we use to cache our target data
   */
  private static class TargetInfo {

    public double yaw = Double.NaN;
    public double pitch = Double.NaN;
    public long timestamp = 0;
  }

  /*
   * Estimate to add to a swerve pose estimator
   */
  public record SwervePoseEstimate(
    Pose2d pose,
    Matrix<N3, N1> stdDevs,
    double timestamp
  ) {}

  // all of these are protected so we can use them in the extended classes
  // which are only extended so we can control which pipelines we are using.

  /** The actual camera object that we get everything from. */
  protected PhotonCamera m_camera;

  /** The result we fetch from PhotonLib each loop. */
  protected List<PhotonPipelineResult> m_results;

  /**
   * The list of TargetInfo objects where we cache all of the target data.
   *
   * <p>Index 0 is the best gamepeice that we detect.
   *
   * <p>Index 1-32 are the AprilTags that are on the field.
   */
  protected final TargetInfo[] m_targetInfo;

  /**
   * The latest estimated pose for this camera
   */
  protected Optional<EstimatedRobotPose> m_poseEstimate = Optional.empty();

  /** The robot origin to camera lens transform 3D that we use to make the pose estimator. */
  protected final Transform3d ROBOT_TO_CAMERA_TRANSFORM; // if this changes, we have bigger issues.

  /** Whether or not we have connection with the camera still */
  protected boolean m_connectionLost;

  /**
   * The fiducial ID of the best target we have.
   *
   * <p>Used for methods that dont take in a fid ID but do some april tag stuff.
   */
  protected int m_bestTargetFiducialId;

  private final PhotonPoseEstimator m_estimator;

  private Matrix<N3, N1> m_multiTagStdDevs;
  private Matrix<N3, N1> m_singleTagStdDevs;
  private Matrix<N3, N1> m_currentStdDevs;

  /**
   * Represents a camera from PhotonVision.
   *
   * <p>Handles connection, caching, calculating stuff, filtering, mostly everything.
   *
   * @param cameraName Name of the cameras Photon Vision network table. MUST match the net tables
   *     name, or it wont work.
   * @param robotToCameraTransform The Transform3d of the robots coordinate center to the camera.
   * @param singleTagStdDevs Standard deviations to use on single-tag strategy
   * @param multiTagStdDevs Standard deviations to use on multi-tag strategt
   */
  public PhotonVisionCamera(
    String cameraName,
    Transform3d robotToCameraTransform,
    Matrix<N3, N1> singleTagStdDevs,
    Matrix<N3, N1> multiTagStdDevs
  ) {
    m_camera = new PhotonCamera(cameraName);
    ROBOT_TO_CAMERA_TRANSFORM = robotToCameraTransform;

    // index 0 is for note detection, 1-16 correspond to apriltag fiducial IDs
    m_targetInfo = new TargetInfo[33];
    for (int i = 0; i < m_targetInfo.length; i++) {
      m_targetInfo[i] = new TargetInfo();
    }

    m_estimator = new PhotonPoseEstimator(
      Constants.FieldConstants.FIELD_LAYOUT,
      robotToCameraTransform
    );

    m_multiTagStdDevs = multiTagStdDevs;
    m_singleTagStdDevs = singleTagStdDevs;

    m_currentStdDevs = VecBuilder.fill(
      Double.MAX_VALUE,
      Double.MAX_VALUE,
      Double.MAX_VALUE
    );
  }

  /**
   * Fetches the latest pipeline result.
   *
   * <p>
   *
   * <h1>YOU SHOULD NEVER CALL THIS! This is for the Robot periodic ONLY. NEVER call this method
   * outside of it. </h1>
   */
  protected void updateResult() {
    if (!m_camera.isConnected() && !m_connectionLost) {
      m_connectionLost = true;
      DriverStation.reportError(
        "[" +
          m_camera.getName() +
          "]\n" +
          PHOTON_VISION.LOST_CONNECTION_ERROR_MESSAGE,
        false
      );
      return;
    }
    m_results = m_camera.getAllUnreadResults();
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (PhotonPipelineResult result : m_results) {
      if (result == null || !result.hasTargets()) {
        return;
      }
      m_bestTargetFiducialId = result.getBestTarget().getFiducialId();
      cacheForAprilTags(result.targets);

      visionEst = m_estimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = m_estimator.estimateLowestAmbiguityPose(result);
      }

      if (visionEst.isEmpty()) {
        return;
      }
      m_poseEstimate = visionEst;
      m_currentStdDevs = updateEstimationStdDevs(
        visionEst.get(),
        result.getTargets()
      );
    }

    if (m_connectionLost) {
      m_connectionLost = false;
      DriverStation.reportWarning(
        "[" +
          m_camera.getName() +
          "]\n" +
          PHOTON_VISION.CONNECTION_REGAINED_MESSAGE,
        false
      );
    }
  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from the tags.
   *
   * From the PhotonVision example code: https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private Matrix<N3, N1> updateEstimationStdDevs(
    EstimatedRobotPose estimatedPose,
    List<PhotonTrackedTarget> targets
  ) {
    // Pose present. Start running Heuristic
    var estStdDevs = m_singleTagStdDevs;
    int numTags = 0;
    double avgDist = 0;

    // Precalculation - see how many tags we found, and calculate an average-distance metric
    for (var tgt : targets) {
      var tagPose = m_estimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist += tagPose
        .get()
        .toPose2d()
        .getTranslation()
        .getDistance(estimatedPose.estimatedPose.toPose2d().getTranslation());
    }

    if (numTags == 0) {
      // No tags visible. Default to single-tag std devs
      return m_singleTagStdDevs;
    } else {
      // One or more tags visible, run the full heuristic.
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = m_multiTagStdDevs;
      // Increase std devs based on (average) distance
      if (numTags == 1 && avgDist > 4) estStdDevs = VecBuilder.fill(
        Double.MAX_VALUE,
        Double.MAX_VALUE,
        Double.MAX_VALUE
      );
      else estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist) / 30));
      return estStdDevs;
    }
  }

  /**
   * The method to cache target data for AprilTags.
   *
   * @param targetList The list of targets that it pulls data from to cache.
   */
  private void cacheForAprilTags(List<PhotonTrackedTarget> targetList) {
    long now = System.currentTimeMillis();
    for (PhotonTrackedTarget targetSeen : targetList) {
      int id = targetSeen.getFiducialId();
      TargetInfo targetInfo = m_targetInfo[id];
      targetInfo.yaw = targetSeen.getYaw();
      targetInfo.pitch = targetSeen.getPitch();
      targetInfo.timestamp = now;
    }
  }

  /**
   * @return Whether or not the camera is connected.
   */
  public boolean isConnected() {
    return m_connectionLost; // uses this because it will be checked every loop
  }

  /**
   * Compares the current system time to the last cached timestamp, and sees if it is older than the
   * passsed in timeout.
   *
   * @param fiducialId Fiducial ID of the desired target to valid the data of. Notes have a
   *     fiducialId of 0
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return If the camera has seen the target within the timeout given
   */
  public boolean isValidTarget(int fiducialId, long timeoutMs) {
    long now = System.currentTimeMillis();
    long then = now - timeoutMs;

    TargetInfo target = m_targetInfo[fiducialId];

    return (
      target.timestamp > then ||
      Math.abs(target.yaw) > PHOTON_VISION.MAX_ANGLE ||
      Math.abs(target.pitch) > PHOTON_VISION.MAX_ANGLE
    );
  }

  /**
   * Sets the pipeline index to make the camera go to.
   *
   * @param index The index to make it be set to.
   */
  public void setPipeline(int index) {
    if (m_camera.getPipelineIndex() != index) {
      m_camera.setPipelineIndex(index);
    }
  }

  /**
   * Gets the pipeline index that an NT subscriber returns.
   *
   * @return The returned pipeline index number.
   */
  public int getPipeline() {
    return m_camera.getPipelineIndex();
  }

  /**
   * @param fiducialId The fiducial ID of the target to get the yaw of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets yaw. <strong>Will be NaN if the cached data was invalid.
   */
  public double getTargetYaw(int fiducialId, long timeoutMs) {
    if (isValidTarget(fiducialId, timeoutMs)) {
      return m_targetInfo[fiducialId].yaw;
    }
    return Double.NaN;
  }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the yaw of the first id in the list, <strong>or NaN if none are valid.
   */
  public double getTargetYaw(int[] fiducialIds, long timeoutMs) {
    for (int id : fiducialIds) {
      double yaw = getTargetYaw(id, timeoutMs);
      if (!Double.isNaN(yaw)) {
        return yaw;
      }
    }
    return Double.NaN;
  }

  /**
   * @param id The ID of the target to get the pitch of.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the desired targets pitch, <strong>will be NaN if the cached data was invalid.
   */
  public double getTargetPitch(int fiducialId, long timeoutMs) {
    if (isValidTarget(fiducialId, timeoutMs)) {
      return m_targetInfo[fiducialId].pitch;
    }
    return Double.NaN;
  }

  /**
   * @param fiducialIds The list of fiducial IDs to check.
   * @param timeoutMs The amount of milliseconds past which target info is deemed expired
   * @return Returns the pitch of the first id in the list, <strong>or NaN if none are valid.
   */
  public double getTargetPitch(int[] fiducialIds, long timeoutMs) {
    for (int id : fiducialIds) {
      double pitch = getTargetPitch(id, timeoutMs);
      if (!Double.isNaN(pitch)) {
        return pitch;
      }
    }
    return Double.NaN;
  }

  public Optional<SwervePoseEstimate> getEstimateForSwerve() {
    if (m_poseEstimate.isEmpty()) {
      return Optional.empty();
    }
    var est = m_poseEstimate.get();
    return Optional.of(
      new SwervePoseEstimate(
        est.estimatedPose.toPose2d(),
        m_currentStdDevs,
        est.timestampSeconds
      )
    );
  }
}
