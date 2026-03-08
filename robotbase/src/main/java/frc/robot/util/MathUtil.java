package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathUtil {

  /**
   * Transforms a velocity along a translation.
   *
   * From: https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/util/geometry/GeomUtil.java#L183
   *
   * Computes the velocity for a new center of rotation given the velocity, new reference pose
   * and current angle
   *
   * Consider the case where the robot is spinning in place. The velocity of the robot's
   * center is zero, but the velocity of a shooter mechanism is in bottom right corner of the robot
   * is of course not zero.
   * This function can compute what the velocity of the shooter mechanism is.
   *
   * @param velocity The original velocity
   * @param transform The transform to the new position
   * @param currentRotation The current rotation of the robot
   * @return The new velocity
   */
  public static ChassisSpeeds transformVelocity(
    ChassisSpeeds velocity,
    Translation2d transform,
    Rotation2d currentRotation
  ) {
    return new ChassisSpeeds(
      velocity.vxMetersPerSecond +
        velocity.omegaRadiansPerSecond *
        (transform.getY() * currentRotation.getCos() -
          transform.getX() * currentRotation.getSin()),
      velocity.vyMetersPerSecond +
        velocity.omegaRadiansPerSecond *
        (transform.getX() * currentRotation.getCos() -
          transform.getY() * currentRotation.getSin()),
      velocity.omegaRadiansPerSecond
    );
  }

  /**
   * Checks if a pose is within a rectangle defined by its bottom right and top left corners.
   *
   * @param rectBottomRight The bottom right corner of the rectangle
   * @param rectTopLeft The top left corner of the rectangle
   * @param pose The pose to check
   * @return True if the pose is within the rectangle, false otherwise
   */
  public static boolean isWithinRect(
    Translation2d rectBottomRight,
    Translation2d rectTopLeft,
    Pose2d pose
  ) {
    return (
      pose.getX() >= rectBottomRight.getX() &&
      pose.getX() <= rectTopLeft.getX() &&
      pose.getY() >= rectBottomRight.getY() &&
      pose.getY() <= rectTopLeft.getY()
    );
  }

  /**
   * Checks if a pose is within a rectangle defined by its center, width, and length.
   *
   * @param rectCenter The center point of the rectangle
   * @param width The width of the rectangle (distance along the y-axis)
   * @param length The length of the rectangle (distance along the x-axis)
   * @param pose The pose to check
   * @return True if the pose is within the rectangle, false otherwise
   */
  public static boolean isWithinRect(
    Translation2d rectCenter,
    double width,
    double length,
    Pose2d pose
  ) {
    double halfWidth = width / 2.0;
    double halfLength = length / 2.0;
    return isWithinRect(
      rectCenter.minus(new Translation2d(halfLength, halfWidth)),
      rectCenter.plus(new Translation2d(halfLength, halfWidth)),
      pose
    );
  }
}
