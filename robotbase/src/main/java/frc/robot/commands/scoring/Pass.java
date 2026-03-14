package frc.robot.commands.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.util.MathUtil;
import java.util.function.Supplier;

public class Pass extends ParallelCommandGroup {

  public Pass(AngularVelocity shooterVelocity, Angle hoodAngle) {
    this(() -> shooterVelocity, () -> hoodAngle);
  }

  public Pass(
    Supplier<AngularVelocity> shooterVelocity,
    Supplier<Angle> hoodAngle
  ) {
    super();
    addCommands(
      Robot.shooter.setVelocity(shooterVelocity),
      Robot.hood.setAngle(hoodAngle),
      new SequentialCommandGroup(
        Robot.shooter.waitUntilTargetVelocity(),
        new ScoreFeed()
      )
      // TODO: Make this work
      // new ConditionalCommand(
      //   new SequentialCommandGroup(
      //     Robot.shooter.waitUntilTargetVelocity(),
      //     new ScoreFeed()
      //   ),
      //   new InstantCommand(),
      //   this::isPositionedToPass
      // )
    );
  }

  /**
   * Checks if the robot is positioned to pass
   * If the robot is too close to the hub, the fuel will most likely collide with it.
   * This makes sure the robot is far enough away so that this does not happen.
   *
   * @return true if the robot is not too close to the hub to pass
   */
  private boolean isPositionedToPass() {
    Pose2d shooterPose = Robot.swerve
      .getAllianceRelativePose2d()
      .transformBy(Constants.SHOOTER.ROBOT_TO_SHOOTER);

    // Current implementation: Makes sure robot is not within a 47" x 47" square directly in front of the hub
    return MathUtil.isWithinRect(
      FieldConstants.Hub.centerPoint,
      FieldConstants.Hub.width,
      FieldConstants.Hub.width,
      shooterPose
    );
  }
}
