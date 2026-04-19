package frc.robot.commands.scoring.teleop;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveTargetLock;
import frc.robot.commands.scoring.ConditionalScoreFeed;
import frc.robot.commands.util.PressToContinue;
import java.util.function.Supplier;

public class TeleopScore extends ParallelCommandGroup {

  public TeleopScore(Dimensionless x, Dimensionless y, Trigger button) {
    this(() -> x, () -> y, button);
  }

  public TeleopScore(
    Supplier<Dimensionless> x,
    Supplier<Dimensionless> y,
    Trigger button
  ) {
    super();
    addCommands(
      Robot.shooter.setVelocity(
        Robot.shotCalculator::getCalculatedShooterVelocity
      ),
      Robot.hood.setAngle(Robot.shotCalculator::getCalculatedHoodAngle),
      new SequentialCommandGroup(
        new WaitUntilCommand(
          Robot.shooter.isAtInitialTargetVelocity()
        ).raceWith(
          new WaitUntilCommand(0.5).alongWith(
            new WaitUntilCommand(() -> !Robot.shotCalculator.isInAllianceZone())
          ),
          new PressToContinue(button)
        ),
        new ConditionalScoreFeed(
          isPositionedToShoot()
            .and(Robot.shooter.isAtContinuousTargetVelocity())
            .and(Robot.shotCalculator.fireControlApproval())
            .or(button)
        )
      ),
      new DriveTargetLock(x, y)
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
          SmartDashboard.putBoolean("is positioned to shoot", false);
          return false;
        }
      }

      SmartDashboard.putBoolean("is positioned to shoot", true);
      return true;
    });
  }
}
