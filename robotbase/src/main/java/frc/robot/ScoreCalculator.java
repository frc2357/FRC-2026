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
public class ScoreCalculator {

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
    public static final Distance CLOSEST_POINT = Inches.of(24); // Up against the hub
    public static final Distance POINT_2 = Inches.of(50);
    public static final Distance POINT_3 = Inches.of(100);
    public static final Distance POINT_4 = Inches.of(200);
    public static final Distance FARTHEST_POINT = Inches.of(265); // Far corner of the outpost
  }

  public ScoreCalculator() {
    m_shooterCurve.put(SHOT_POINTS.CLOSEST_POINT, RotationsPerSecond.of(48.5));
    m_shooterCurve.put(SHOT_POINTS.POINT_2, RotationsPerSecond.of(48.5));
    m_shooterCurve.put(SHOT_POINTS.POINT_3, RotationsPerSecond.of(53));
    m_shooterCurve.put(SHOT_POINTS.POINT_4, RotationsPerSecond.of(64));
    m_shooterCurve.put(SHOT_POINTS.FARTHEST_POINT, RotationsPerSecond.of(64));

    m_hoodCurve.put(SHOT_POINTS.CLOSEST_POINT, Degrees.of(0));
    m_hoodCurve.put(SHOT_POINTS.POINT_2, Degrees.of(0));
    m_hoodCurve.put(SHOT_POINTS.POINT_3, Degrees.of(2));
    m_hoodCurve.put(SHOT_POINTS.POINT_4, Degrees.of(14));
    m_hoodCurve.put(SHOT_POINTS.FARTHEST_POINT, Degrees.of(14));

    m_timeOfFlightCurve.put(SHOT_POINTS.CLOSEST_POINT, Seconds.of(0));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_2, Seconds.of(0));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_3, Seconds.of(0));
    m_timeOfFlightCurve.put(SHOT_POINTS.POINT_4, Seconds.of(0));
    m_timeOfFlightCurve.put(SHOT_POINTS.FARTHEST_POINT, Seconds.of(0));

    SmartDashboard.putBoolean(SCORING.IS_SOTF_KEY, false);
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

    Translation2d targetHub = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );

    Distance targetDistance = Meters.of(
      shooterPose.getTranslation().getDistance(targetHub)
    );
    SmartDashboard.putNumber(
      "Pose Computed Distance Inches",
      targetDistance.in(Inches)
    );

    AngularVelocity shooterVelocity = m_shooterCurve.get(targetDistance);
    Angle hoodAngle = m_hoodCurve.get(targetDistance);
    Rotation2d driveAngle = computeTargetDriveAngle(robotPose, targetHub);

    SmartDashboard.putNumber(
      "Computed Shooter RPS",
      shooterVelocity.in(RotationsPerSecond)
    );
    SmartDashboard.putNumber("Computed Hood Angle", hoodAngle.in(Degrees));
    SmartDashboard.putNumber("Computed Drive Angle", driveAngle.getDegrees());

    return new CalculatedShot(shooterVelocity, hoodAngle, driveAngle);
  }

  /**
   * Implementation of LUT based shoot on the fly algorithm
   * @return
   */
  public CalculatedShot calculateShotFromShootOnTheFly() {
    // Get current robot pose
    Pose2d initialRobotPose = Robot.swerve.getFieldRelativePose2d();
    ChassisSpeeds robotSpeeds = Robot.swerve.getCurrentFieldRelativeSpeeds();

    // Account for the robot's velocity and latency compensation
    // to compute a guess to where the robot actually is
    Pose2d velocityCompensatedRobotPose = initialRobotPose.exp(
      new Twist2d(
        robotSpeeds.vxMetersPerSecond *
          SCORING.SOTF_LATENCY_COMPENSATION.in(Seconds),
        robotSpeeds.vyMetersPerSecond *
          SCORING.SOTF_LATENCY_COMPENSATION.in(Seconds),
        robotSpeeds.omegaRadiansPerSecond *
          SCORING.SOTF_LATENCY_COMPENSATION.in(Seconds)
      )
    );

    Pose2d shooterPose = velocityCompensatedRobotPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER
    );

    Translation2d targetHub = AllianceFlipUtil.apply(
      FieldConstants.Hub.topCenterPoint.toTranslation2d()
    );

    // Initial target distance
    Distance shooterToTargetDistance = Meters.of(
      targetHub.getDistance(shooterPose.getTranslation())
    );

    // The field-relative speed of the shooter moving on the field
    ChassisSpeeds shooterSpeeds = MathUtil.transformVelocity(
      robotSpeeds,
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
        targetHub.getDistance(futureShooterPose.getTranslation())
      );
    }

    AngularVelocity shooterVelocity = m_shooterCurve.get(
      futureShootertoTargetDistance
    );
    Angle hoodAngle = m_hoodCurve.get(futureShootertoTargetDistance);

    Pose2d futureRobotPose = futureShooterPose.transformBy(
      Constants.SHOOTER.ROBOT_TO_SHOOTER.inverse()
    );
    Rotation2d driveAngle = computeTargetDriveAngle(futureRobotPose, targetHub);

    SmartDashboard.putNumber(
      "SOTF Direct Distance Inches",
      shooterToTargetDistance.in(Inches)
    );
    SmartDashboard.putNumber(
      "SOTF Future Distance Inches",
      futureShootertoTargetDistance.in(Inches)
    );
    SmartDashboard.putNumber(
      "SOTF Shooter RPS",
      shooterVelocity.in(RotationsPerSecond)
    );
    SmartDashboard.putNumber("SOTF Hood Angle", hoodAngle.in(Degrees));
    SmartDashboard.putNumber("SOTF Drive Angle", driveAngle.getDegrees());

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

    // Run 2 iterations to account for the launcher 'swinging' as the robot rotates
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

  /**
   * Should be called in Robot.periodic every loop
   */
  public void updateCalculatedShot() {
    if (SmartDashboard.getBoolean(SCORING.IS_SOTF_KEY, false)) {
      m_latestCalculatedShot = calculateShotFromShootOnTheFly();
    } else {
      m_latestCalculatedShot = calculateShotFromPoseStationary();
    }
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
    return m_shooterCurve.get(distance);
  }

  public Angle getHoodAngleStationary(Distance distance) {
    return m_hoodCurve.get(distance);
  }

  public void updateCurveTuners() {
    m_shooterCurve.updateCurveValues();
    m_hoodCurve.updateCurveValues();
  }

  public void logCurveValues() {
    m_shooterCurve.logCurrentValues();
    m_hoodCurve.logCurrentValues();
  }
}
