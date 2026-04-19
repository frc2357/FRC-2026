package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;

public interface CameraInterface {
  public abstract void updateResult();
  public abstract Optional<SwervePoseEstimate> getEstimateForSwerve();
  public abstract void setPipeline(VisionPipeline pipeline);

  /*
   * Estimate to add to a swerve pose estimator
   */
  public record SwervePoseEstimate(
    Pose2d pose,
    Matrix<N3, N1> stdDevs,
    double timestamp
  ) {}
}
