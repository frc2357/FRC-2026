package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.Supplier;

public class Score extends ParallelCommandGroup {

  public Score(AngularVelocity shooterVelocity, Angle hoodAngle) {
    this(() -> shooterVelocity, () -> hoodAngle);
  }

  public Score(
    Supplier<AngularVelocity> shooterVelocity,
    Supplier<Angle> hoodAngle
  ) {
    super();
    addCommands(
      Robot.shooter.setVelocity(shooterVelocity),
      Robot.hood.setAngle(hoodAngle),
      new ConditionalScoreFeed(
        isPositionedToShoot().and(Robot.shooter.isAtTargetVelocity())
      )
    );
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
