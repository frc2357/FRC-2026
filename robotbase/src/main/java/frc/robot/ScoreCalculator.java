package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.InterpolationUtil;

public class ScoreCalculator {

  public record CalculatedShot(
    AngularVelocity shooterVelocity,
    Angle hoodPosition
  ) {}

  private final InterpolatingTreeMap<Distance, AngularVelocity> m_shooterCurve =
    new InterpolatingTreeMap<Distance, AngularVelocity>(
      InterpolationUtil::InverseInterpolate,
      InterpolationUtil::Interpolate
    );

  private final InterpolatingTreeMap<Distance, Angle> m_hoodCurve =
    new InterpolatingTreeMap<Distance, Angle>(
      InterpolationUtil::InverseInterpolate,
      InterpolationUtil::Interpolate
    );

  public static final class SHOT_POINTS {

    // TODO: Rename to be more indicative of the point
    // Distances should be relative to top center point of the hub
    public static final Distance CLOSEST_POINT = Inches.of(24); // Up against the hub
    public static final Distance POINT_2 = Inches.of(5);
    public static final Distance POINT_3 = Inches.of(5);
    public static final Distance POINT_4 = Inches.of(5);
    public static final Distance FARTHEST_POINT = Inches.of(265); // Far corner of the outpost
  }

  public ScoreCalculator() {
    m_shooterCurve.put(SHOT_POINTS.CLOSEST_POINT, RPM.of(1000));
    m_shooterCurve.put(SHOT_POINTS.POINT_2, RPM.of(2000));
    m_shooterCurve.put(SHOT_POINTS.POINT_3, RPM.of(3000));
    m_shooterCurve.put(SHOT_POINTS.POINT_4, RPM.of(4000));
    m_shooterCurve.put(SHOT_POINTS.FARTHEST_POINT, RPM.of(5000));

    m_hoodCurve.put(SHOT_POINTS.CLOSEST_POINT, Degrees.of(0));
    m_hoodCurve.put(SHOT_POINTS.POINT_2, Degrees.of(0));
    m_hoodCurve.put(SHOT_POINTS.POINT_3, Degrees.of(0));
    m_hoodCurve.put(SHOT_POINTS.POINT_4, Degrees.of(0));
    m_hoodCurve.put(SHOT_POINTS.FARTHEST_POINT, Degrees.of(0));
  }

  public CalculatedShot calculateShot(Distance targetDistance) {
    AngularVelocity shooterVelocity = m_shooterCurve.get(targetDistance);
    Angle hoodAngle = m_hoodCurve.get(targetDistance);

    SmartDashboard.putNumber("Computed Shooter RPM", shooterVelocity.in(RPM));
    SmartDashboard.putNumber("Computed Hood Angle", hoodAngle.in(Degrees));

    return new CalculatedShot(shooterVelocity, hoodAngle);
  }

  public CalculatedShot calculateFromPose() {
    return calculateShot(shotDistanceFromPose());
  }

  /**
   * Computed using pose estimation
   * @return The distance between the top center point of the hub and the shooter flywheel
   */
  public Distance shotDistanceFromPose() {
    Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();

    Pose2d shooterPose = robotPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER
    );

    Translation2d targetHub = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );

    Distance distance = Meters.of(
      shooterPose.getTranslation().getDistance(targetHub)
    );
    SmartDashboard.putNumber(
      "Pose Computed Distance Inches",
      distance.in(Inches)
    );
    return distance;
  }

  public CalculatedShot calculatedFromShooterCamera() {
    Distance distance = shotDistanceFromShooterCamera();
    return calculateShot(distance);
  }

  // TODO Implement, requires changes in pose initialization branch
  // Pre-emptive work in the event pose estimation proves to be un-reliable
  /**
   * Computed using the pitch and yaw of the shooter camera
   * @return The distance between the top center point of the hub and the shooter flywheel
   */
  public Distance shotDistanceFromShooterCamera() {
    return Meters.of(0);
  }
}
