package frc.robot.commands.scoring.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.scoring.ConditionalScoreFeed;
import java.util.function.Supplier;

public class TeleopScore extends ParallelCommandGroup {

  public TeleopScore(AngularVelocity shooterVelocity, Angle hoodAngle) {
    this(() -> shooterVelocity, () -> hoodAngle);
  }

  public TeleopScore(
    Supplier<AngularVelocity> shooterVelocity,
    Supplier<Angle> hoodAngle
  ) {
    super();
    addCommands(
      Robot.shooter.setVelocity(shooterVelocity),
      Robot.hood.setAngle(hoodAngle),
      new SequentialCommandGroup(
        new WaitUntilCommand(Robot.shooter.isAtInitialTargetVelocity()),
        new ConditionalScoreFeed(
          isPositionedToShoot().and(
            Robot.shooter.isAtContinuousTargetVelocity()
          )
        )
      )
    );
  }

  // needs more work
  private Trigger isAngledToShoot() {
    return new Trigger(() -> {
      Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();

      return robotPose
        .getRotation()
        .getMeasure()
        .isNear(
          Robot.shotCalculator.getCalculatedDriveAngle().getMeasure(),
          Constants.SWERVE.TELEOP_SHOOT_DRIVE_ANGLE_TOLERANCE
        );
    });
  }

  private Trigger isPositionedToShoot() {
    return new Trigger(() -> {
      Pose2d robotPose = Robot.swerve.getFieldRelativePose2d();
      Pose2d shooterPose = robotPose.transformBy(
        Constants.SHOOTER.ROBOT_TO_SHOOTER
      );

      for (Rectangle2d zone : Constants.SCORING.NO_SHOOT_ZONES) {
        if (zone.contains(shooterPose.getTranslation())) {
          return false;
        }
      }

      return true;
    });
  }
}
