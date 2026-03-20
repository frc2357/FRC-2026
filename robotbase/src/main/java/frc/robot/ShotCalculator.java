package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SCORING;
import frc.robot.networkTables.CurveTuner;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.InterpolationUtil;
import frc.robot.util.MathUtil;

/**
 * Performs various computations to calculate parameters for scoring into the hub
 *
 * This file has a lot of object instantiation, if we start seeing occasional
 * loop overruns, this is a point of optimization.
 */
public class ShotCalculator {

  private Field2d targetField = new Field2d();

  public record CalculatedShot(
    AngularVelocity shooterVelocity,
    Angle hoodPosition,
    Rotation2d driveAngle
  ) {}

  private CalculatedShot m_latestCalculatedShot = new CalculatedShot(
    RotationsPerSecond.of(0),
    Degrees.of(0),
    Rotation2d.fromDegrees(0)
  );

  private final CurveTuner<Distance, AngularVelocity> m_passingShooterCurve =
    new CurveTuner<Distance, AngularVelocity>(
      "Passing Shooter Curve",
      InterpolationUtil::InverseInterpolate,
      InterpolationUtil::Interpolate
    );

  private final CurveTuner<Distance, Angle> m_passingHoodCurve = new CurveTuner<
    Distance,
    Angle
  >(
    "Passing Hood Curve",
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  private final CurveTuner<Distance, AngularVelocity> m_shooterCurve =
    new CurveTuner<Distance, AngularVelocity>(
      "Shooter Curve",
      InterpolationUtil::InverseInterpolate,
      InterpolationUtil::Interpolate
    );

  private final CurveTuner<Distance, Angle> m_hoodCurve = new CurveTuner<
    Distance,
    Angle
  >(
    "Hood Curve",
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  private final CurveTuner<Distance, Time> m_timeOfFlightCurve = new CurveTuner<
    Distance,
    Time
  >(
    "ToF Curve",
    InterpolationUtil::InverseInterpolate,
    InterpolationUtil::Interpolate
  );

  public static final class SHOT_POINTS {

    // TODO: Rename to be more indicative of the point
    // Distances should be relative to top center point of the hub
    public static final Distance HUB = Inches.of(43); // Up against the hub
    public static final Distance POINT_2 = Inches.of(75);
    public static final Distance POINT_3 = Inches.of(100);
    public static final Distance TRENCH = Inches.of(127);
    public static final Distance POINT_5 = Inches.of(150);
    public static final Distance POINT_6 = Inches.of(175);
    public static final Distance OUTPOST_CORNER = Inches.of(205); // Far corner of the outpost
  }

  public static final class PASS_POINTS {

    // These points are not exact and don't correlate to anything specific. They are mainly here for simpler interpolation.
    public static final Distance CLOSEST = Inches.of(43);
    public static final Distance CEILING = Inches.of(250);
    public static final Distance FURTHERST = Inches.of(500);
  }

  public ShotCalculator() {
    m_passingShooterCurve.put(PASS_POINTS.CLOSEST, RotationsPerSecond.of(50));
    //m_passingShooterCurve.put(PASS_POINTS.CEILING, RotationsPerSecond.of(95));
    m_passingShooterCurve.put(PASS_POINTS.FURTHERST, RotationsPerSecond.of(95));

    m_passingHoodCurve.put(PASS_POINTS.CLOSEST, Degrees.of(18));
    m_passingHoodCurve.put(PASS_POINTS.CEILING, Degrees.of(34));
    m_passingHoodCurve.put(PASS_POINTS.FURTHERST, Degrees.of(34));

    m_shooterCurve.put(SHOT_POINTS.HUB, RotationsPerSecond.of(50));
    m_shooterCurve.put(SHOT_POINTS.POINT_2, RotationsPerSecond.of(52));
    m_shooterCurve.put(SHOT_POINTS.POINT_3, RotationsPerSecond.of(54));
    m_shooterCurve.put(SHOT_POINTS.TRENCH, RotationsPerSecond.of(56));
    m_shooterCurve.put(SHOT_POINTS.POINT_5, RotationsPerSecond.of(60));
    m_shooterCurve.put(SHOT_POINTS.POINT_6, RotationsPerSecond.of(63));
    m_shooterCurve.put(SHOT_POINTS.OUTPOST_CORNER, RotationsPerSecond.of(66));

    m_hoodCurve.put(SHOT_POINTS.HUB, Degrees.of(1));
    m_hoodCurve.put(SHOT_POINTS.POINT_2, Degrees.of(5));
    m_hoodCurve.put(SHOT_POINTS.POINT_3, Degrees.of(8));
    m_hoodCurve.put(SHOT_POINTS.TRENCH, Degrees.of(11));
    m_hoodCurve.put(SHOT_POINTS.POINT_5, Degrees.of(14));
    m_hoodCurve.put(SHOT_POINTS.POINT_6, Degrees.of(17));
    m_hoodCurve.put(SHOT_POINTS.OUTPOST_CORNER, Degrees.of(18));

    m_timeOfFlightCurve.put(SHOT_POINTS.HUB, Seconds.of(1.005));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_2, Seconds.of(1.068));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_3, Seconds.of(1.168));
    m_timeOfFlightCurve.put(SHOT_POINTS.TRENCH, Seconds.of(1.132));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_5, Seconds.of(1.032));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_6, Seconds.of(1.138));
    m_timeOfFlightCurve.put(SHOT_POINTS.OUTPOST_CORNER, Seconds.of(1.18));

    SmartDashboard.putData("target", targetField);
  }

  /**
   * Computed using pose estimation
   * @return The distance between the top center point of the hub and the shooter flywheel
   */
  public CalculatedShot calculateShotFromPoseStationary() {
    Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();

    Pose2d shooterPose = robotPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER
    );

    Translation2d target = AllianceFlipUtil.apply(getShotTarget());

    Distance targetDistance = Meters.of(
      shooterPose.getTranslation().getDistance(target)
    );

    AngularVelocity shooterVelocity;
    Angle hoodAngle;
    if (isInAllianceZone()) {
      shooterVelocity = m_shooterCurve.get(targetDistance);
      hoodAngle = m_hoodCurve.get(targetDistance);
    } else {
      shooterVelocity = m_passingShooterCurve.get(targetDistance);
      hoodAngle = m_passingHoodCurve.get(targetDistance);
    }
    Rotation2d driveAngle = computeTargetDriveAngle(robotPose, target);

    return new CalculatedShot(shooterVelocity, hoodAngle, driveAngle);
  }

  /**
   * Implementation of LUT based shoot on the fly algorithm
   * @return
   */
  public CalculatedShot calculateShotFromShootOnTheFly() {
    // Get current robot pose
    Pose2d initialRobotPose = Robot.swerve.getFieldRelativePose2d();
    ChassisSpeeds robotRelativeSpeeds =
      Robot.swerve.getCurrentRobotRelativeSpeeds();
    ChassisSpeeds fieldRelativeSpeeds =
      Robot.swerve.getCurrentFieldRelativeSpeeds(); // try field velocities instead

    var latency = Constants.SCORING.SOTF_LATENCY_COMPENSATION.in(Seconds);
    // Account for the robot's velocity and latency compensation
    // to compute a guess to where the robot actually is
    Pose2d velocityCompensatedRobotPose = initialRobotPose.exp(
      new Twist2d(
        robotRelativeSpeeds.vxMetersPerSecond * latency,
        robotRelativeSpeeds.vyMetersPerSecond * latency,
        robotRelativeSpeeds.omegaRadiansPerSecond * latency
      )
    );

    Pose2d shooterPose = velocityCompensatedRobotPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER
    );

    Translation2d target = AllianceFlipUtil.apply(getShotTarget());

    // Initial target distance
    Distance shooterToTargetDistance = Meters.of(
      target.getDistance(shooterPose.getTranslation())
    );

    SmartDashboard.putNumber(
      "Direct Distance Inches",
      shooterToTargetDistance.in(Inches)
    );

    // The field-relative speed of the shooter moving on the field
    ChassisSpeeds shooterSpeeds = MathUtil.transformVelocity(
      fieldRelativeSpeeds,
      Constants.SHOOTER.ROBOT_TO_SHOOTER.getTranslation(),
      initialRobotPose.getRotation()
    );

    // Account for robot velocity and compute future target distance
    Time timeOfFlight = m_timeOfFlightCurve.get(shooterToTargetDistance);
    Pose2d futureShooterPose = shooterPose;
    Distance futureShootertoTargetDistance = shooterToTargetDistance;

    // Converge on the future position
    // increasing iterations will improve accuracy but decrease performance
    // decreasing iterations will reduce accuracy but increase performance
    // it is likely jut a few iterations will produce a result good enough for us
    for (int i = 0; i < SCORING.SOTF_CONVERGE_ITERATIONS; i++) {
      timeOfFlight = m_timeOfFlightCurve.get(futureShootertoTargetDistance);
      double offsetX =
        shooterSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
      double offsetY =
        shooterSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);
      futureShooterPose = new Pose2d(
        shooterPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
        shooterPose.getRotation()
      );
      futureShootertoTargetDistance = Meters.of(
        target.getDistance(futureShooterPose.getTranslation())
      );
    }

    // Should use a moving average to compute at least
    // hood angle and drive angle. Maybe shooter velocity too
    AngularVelocity shooterVelocity;
    Angle hoodAngle;
    if (isInAllianceZone()) {
      shooterVelocity = m_shooterCurve.get(futureShootertoTargetDistance);
      hoodAngle = m_hoodCurve.get(futureShootertoTargetDistance);
    } else {
      shooterVelocity = m_passingShooterCurve.get(
        futureShootertoTargetDistance
      );
      hoodAngle = Constants.HOOD.PASSING_STATIC_ANGLE;
    }

    Pose2d futureRobotPose = futureShooterPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER.inverse()
    );
    Rotation2d driveAngle = computeTargetDriveAngle(futureRobotPose, target);

    targetField.setRobotPose(
      target.getX(),
      target.getY(),
      Rotation2d.fromDegrees(0)
    );

    SmartDashboard.putNumber("target angle", driveAngle.getDegrees());
    return new CalculatedShot(shooterVelocity, hoodAngle, driveAngle);
  }

  // TODO Implement, requires changes in pose initialization branch
  // Pre-emptive work in the event pose estimation proves to be un-reliable
  public CalculatedShot calculatedFromShooterCameraStationary() {
    return new CalculatedShot(
      RotationsPerSecond.of(0),
      Degrees.of(0),
      Rotation2d.fromDegrees(0)
    );
  }

  private static Rotation2d computeTargetDriveAngle(
    Pose2d robotPose,
    Translation2d target
  ) {
    // Start with the angle from robot center to target
    Rotation2d driveAngleGuess = target
      .minus(robotPose.getTranslation())
      .getAngle();

    // Run iterations to account for the launcher 'swinging' as the robot rotates
    for (int i = 0; i < SCORING.DRIVE_ANGLE_CONVERGE_ITERATIONS; i++) {
      // 1. Find where the launcher would be on the field if the robot faces our guess
      Translation2d projectedLauncherPose = robotPose
        .getTranslation()
        .plus(
          Constants.SHOOTER.ROBOT_TO_SHOOTER.getTranslation().rotateBy(
            driveAngleGuess
          )
        );

      // 2. Find the angle from that projected launcher position to the target
      Rotation2d launcherToTargetAngle = target
        .minus(projectedLauncherPose)
        .getAngle();

      // 3. Subtract the launcher's mount angle from the required pointing angle
      // If launcher is at -50, robot must be at (TargetAngle - (-50))
      driveAngleGuess = launcherToTargetAngle.minus(
        Constants.SHOOTER.ROBOT_TO_SHOOTER.getRotation()
      );
    }

    return driveAngleGuess;
  }

  public boolean isInAllianceZone() {
    Pose2d robotPose = Robot.swerve.getAllianceRelativePose2d();
    Pose2d shooterPose = robotPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER
    );
    return shooterPose.getX() < FieldConstants.LinesVertical.allianceZone;
  }

  private Translation2d getShotTarget() {
    Pose2d robotPose = Robot.swerve.getAllianceRelativePose2d();
    Pose2d shooterPose = robotPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER
    );

    if (isInAllianceZone()) {
      return FieldConstants.Hub.centerPoint;
    } else {
      if (shooterPose.getY() > FieldConstants.Hub.centerPoint.getY()) {
        return FieldConstants.LeftBump.centerPoint;
      } else {
        return FieldConstants.RightBump.centerPoint;
      }
    }
  }

  /**
   * Should be called in Robot.periodic every loop
   */
  public void updateCalculatedShot() {
    m_latestCalculatedShot = calculateShotFromShootOnTheFly();
  }

  public CalculatedShot getCalculatedShot() {
    return m_latestCalculatedShot;
  }

  public Rotation2d getCalculatedDriveAngle() {
    return m_latestCalculatedShot.driveAngle();
  }

  public AngularVelocity getCalculatedShooterVelocity() {
    return m_latestCalculatedShot.shooterVelocity();
  }

  public Angle getCalculatedHoodAngle() {
    return m_latestCalculatedShot.hoodPosition();
  }

  public AngularVelocity getShooterVelocityStationary(Distance distance) {
    if (isInAllianceZone()) {
      return m_shooterCurve.get(distance);
    } else {
      return m_passingShooterCurve.get(distance);
    }
  }

  public Angle getHoodAngleStationary(Distance distance) {
    if (isInAllianceZone()) {
      return m_hoodCurve.get(distance);
    } else {
      return m_passingHoodCurve.get(distance);
    }
  }

  public void updateCurveTuners() {
    m_shooterCurve.updateCurveValues();
    m_hoodCurve.updateCurveValues();
    m_passingShooterCurve.updateCurveValues();
    m_passingHoodCurve.updateCurveValues();
  }

  public void logCurveValues() {
    m_shooterCurve.logCurrentValues();
    m_hoodCurve.logCurrentValues();
    m_passingShooterCurve.logCurrentValues();
    m_passingHoodCurve.logCurrentValues();
  }
}
