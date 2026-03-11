package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class StaticScore extends ParallelCommandGroup {

  public StaticScore(Angle angle, AngularVelocity rps) {
    super(
      Robot.hood.setAngle(angle),
      Robot.shooter.setVelocity(rps),
      new SequentialCommandGroup(
        Robot.shooter.waitUntilTargetVelocity(),
        new ScoreFeed()
      )
    );
  }
}
