package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drive.DrivePoseTargetingHub;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.floor.FloorSetSpeed;
import java.util.function.Supplier;

public class VisionScore extends ParallelCommandGroup {

  public VisionScore(Supplier<Dimensionless> x, Supplier<Dimensionless> y) {
    super(
      new VisionTargeting(),
      new DrivePoseTargetingHub(x, y),
      new SequentialCommandGroup(
        Robot.shooter.waitUntilTargetVelocity(),
        new ParallelCommandGroup(
          new FeederSetSpeed(Constants.FEEDER.FEED_SPEED),
          new FloorSetSpeed(Constants.FLOOR.FLOOR_SPEED)
        )
      )
    );
  }
}
