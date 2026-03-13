package frc.robot.commands.passing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePosePassing;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.floor.FloorStop;
import frc.robot.util.MathUtil;
import java.util.function.Supplier;

public class VisionPass extends ParallelCommandGroup {

  public VisionPass(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super();
    addCommands(
      new VisionTargetPassing(),
      new DrivePosePassing(x, y),
      new SequentialCommandGroup(
        Robot.shooter.waitUntilTargetVelocity(),
        new ConditionalCommand(
          new ParallelCommandGroup(
            new FeederSetSpeed(Constants.FEEDER.FEED_SPEED),
            new FloorSetSpeed(Constants.FLOOR.FLOOR_SPEED)
          ),
          new ParallelCommandGroup(new FeederStop(), new FloorStop()),
          this::isPositionedToPass
        )
      )
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
