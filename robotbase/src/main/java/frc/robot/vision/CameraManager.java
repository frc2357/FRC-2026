package frc.robot.vision;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PHOTON_VISION;
import frc.robot.vision.PhotonVisionCamera.SwervePoseEstimate;
import java.util.Optional;
import java.util.function.Consumer;

public class CameraManager extends SubsystemBase {

  // TODO: Switch to actual shooter camera
  public PhotonVisionCamera m_shooter = new PhotonVisionCamera(
    Constants.PHOTON_VISION.SHOOTER_CAM.NAME,
    Constants.PHOTON_VISION.SHOOTER_CAM.ROBOT_TO_CAM_TRANSFORM,
    Constants.PHOTON_VISION.SHOOTER_CAM.kSingleTagStdDevs,
    Constants.PHOTON_VISION.SHOOTER_CAM.kMultiTagStdDevs
  );

  // TODO: Enable these and add to m_cameras when we are on the robot
  // public PhotonVisionCamera m_backLeft = new PhotonVisionCamera(
  //   Constants.PHOTON_VISION.BACK_LEFT_CAM.NAME,
  //   Constants.PHOTON_VISION.BACK_LEFT_CAM.ROBOT_TO_CAM_TRANSFORM,
  //   Constants.PHOTON_VISION.BACK_LEFT_CAM.kSingleTagStdDevs,
  //   Constants.PHOTON_VISION.BACK_LEFT_CAM.kMultiTagStdDevs
  // );
  // public PhotonVisionCamera m_backRight = new PhotonVisionCamera(
  //   Constants.PHOTON_VISION.BACK_RIGHT_CAM.NAME,
  //   Constants.PHOTON_VISION.BACK_RIGHT_CAM.ROBOT_TO_CAM_TRANSFORM,
  //   Constants.PHOTON_VISION.BACK_RIGHT_CAM.kSingleTagStdDevs,
  //   Constants.PHOTON_VISION.BACK_RIGHT_CAM.kMultiTagStdDevs
  // );

  PhotonVisionCamera[] m_cameras = { m_shooter };

  @SuppressWarnings("unchecked")
  Optional<SwervePoseEstimate>[] m_estimates = (Optional<
    SwervePoseEstimate
  >[]) new Optional<?>[m_cameras.length];

  private static final Field2d m_visionField = new Field2d();

  public CameraManager() {
    SmartDashboard.putData("Vision Field", m_visionField);
    // Set the pipeline, only really necessary if we were testing with another pipeline in the UI beforehand
    setPipeline(PHOTON_VISION.MULTI_TAG_PIPELINE);
  }

  public void updateResult() {
    for (int i = 0; i < m_cameras.length; i++) {
      m_cameras[i].updateResult();
      m_estimates[i] = m_cameras[i].getEstimateForSwerve();
    }
  }

  public Optional<SwervePoseEstimate>[] getEstimates() {
    return m_estimates;
  }

  public void addSwerveEstimates(Consumer<SwervePoseEstimate> swerveEstimator) {
    for (Optional<SwervePoseEstimate> optionalEstimate : m_estimates) {
      optionalEstimate.ifPresent(swerveEstimator);
    }
  }

  private void setPipeline(int pipelineIndex) {
    for (PhotonVisionCamera camera : m_cameras) {
      camera.setPipeline(pipelineIndex);
    }
  }

  @Override
  public void periodic() {
    this.addSwerveEstimates((SwervePoseEstimate estimate) ->
      m_visionField.setRobotPose(estimate.pose())
    );
  }
}
