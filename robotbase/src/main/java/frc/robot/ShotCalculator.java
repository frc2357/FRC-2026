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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SCORING;
import frc.robot.ShiftTimer.ShiftInfo;
import frc.robot.networkTables.CurveTuner;
import frc.robot.util.AllianceFlipUtil;
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
    Rotation2d driveAngle,
    Time timeOfFlight
  ) {}

  private CalculatedShot m_latestCalculatedShot = new CalculatedShot(
    RotationsPerSecond.of(0),
    Degrees.of(0),
    Rotation2d.fromDegrees(0),
    Seconds.of(0)
  );

  public ShotCalculator() {
    SmartDashboard.putData("target", targetField);
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
    Time timeOfFlight = Robot.shooterCurveManager.getTimeOfFlight(
      shooterToTargetDistance
    );
    Pose2d futureShooterPose = shooterPose;
    Distance futureShootertoTargetDistance = shooterToTargetDistance;

    // Converge on the future position
    // increasing iterations will improve accuracy but decrease performance
    // decreasing iterations will reduce accuracy but increase performance
    // it is likely jut a few iterations will produce a result good enough for us
    for (int i = 0; i < SCORING.SOTF_CONVERGE_ITERATIONS; i++) {
      timeOfFlight = Robot.shooterCurveManager.getTimeOfFlight(
        futureShootertoTargetDistance
      );
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
      shooterVelocity = Robot.shooterCurveManager.getScoringShooterVelocity(
        futureShootertoTargetDistance
      );
      hoodAngle = Robot.shooterCurveManager.getScoringHoodAngle(
        futureShootertoTargetDistance
      );
    } else {
      shooterVelocity = Robot.shooterCurveManager.getPassingShooterVelocity(
        futureShootertoTargetDistance
      );
      hoodAngle = Robot.shooterCurveManager.getPassingHoodAngle(
        futureShootertoTargetDistance
      );
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

    return new CalculatedShot(
      shooterVelocity,
      hoodAngle,
      driveAngle,
      timeOfFlight
    );
  }

  // TODO Implement, requires changes in pose initialization branch
  // Pre-emptive work in the event pose estimation proves to be un-reliable
  public CalculatedShot calculatedFromShooterCameraStationary() {
    return new CalculatedShot(
      RotationsPerSecond.of(0),
      Degrees.of(0),
      Rotation2d.fromDegrees(0),
      Seconds.of(0)
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

  public Trigger fireControlApproval() {
    return new Trigger(() -> {
      if (!DriverStation.isFMSAttached()) {
        return true;
      }
      if (!isInAllianceZone()) {
        // Always approves when passing
        return true;
      }

      ShiftInfo shiftInfo = Robot.shiftTimer.getShiftInfo();

      if (shiftInfo.isHubActive()) {
        // Always approves if the hub is active
        return true;
      }

      if (
        (shiftInfo
            .timeRemaining()
            .lte(
              getCalculatedShot().timeOfFlight.minus(
                Constants.SCORING.TOF_TIMING_BUFFER
              )
            ))
      ) {
        /* if the hub is not active, BUT the remaining time until activation is LESS THAN the time of flight,
         it will return true */
        return true;
      } else return false;
    });
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
}
