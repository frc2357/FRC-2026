package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SHOOTER;
import frc.robot.networkTables.CurveTuner;
import frc.robot.util.InterpolationUtil;

public class PassCalculator {

  private AngularVelocity m_shooterVelocity;

  private final CurveTuner<Distance, AngularVelocity> m_shooterCurve =
    new CurveTuner<>(
      "Passing Shooter Curve",
      InterpolationUtil::InverseInterpolate,
      InterpolationUtil::Interpolate
    );

  public static final class PASS_POINTS {

    public static final Distance CLOSEST_POINT = Inches.of(47);
    public static final Distance FURTHEST_POINT = Inches.of(265);
  }

  public PassCalculator() {
    m_shooterCurve.put(PASS_POINTS.CLOSEST_POINT, RotationsPerSecond.of(40));
    m_shooterCurve.put(PASS_POINTS.FURTHEST_POINT, RotationsPerSecond.of(70));
  }

  public AngularVelocity calculatePassingAngularVelocity() {
    Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();
    Pose2d shooterPose = robotPose.transformBy(SHOOTER.ROBOT_TO_SHOOTER);

    Translation2d target = getTarget();
    Distance targetDistance = Meters.of(
      target.getDistance(shooterPose.getTranslation())
    );

    return m_shooterCurve.get(targetDistance);
  }

  private Translation2d getTarget() {
    Pose2d robotPose = Robot.swerve.getAllianceRelativePose2d();
    if (robotPose.getY() > FieldConstants.Hub.centerPoint.getY()) {
      return FieldConstants.Bump.Left.centerPoint;
    } else {
      return FieldConstants.Bump.Right.centerPoint;
    }
  }

  public void updateCalculatedShot() {
    m_shooterVelocity = calculatePassingAngularVelocity();
  }

  public AngularVelocity getCalculatedShooterVelocity() {
    return m_shooterVelocity;
  }

  public void updateCurveTuners() {
    m_shooterCurve.updateCurveValues();
  }

  public void logCurveValues() {
    m_shooterCurve.logCurrentValues();
  }
}
