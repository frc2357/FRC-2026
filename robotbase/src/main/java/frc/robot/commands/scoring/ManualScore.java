package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class ManualScore extends ParallelCommandGroup {

  public ManualScore(AngularVelocity rpm) {
    super(
      Robot.shooter.setVelocity(rpm),
      new SequentialCommandGroup(
        Robot.shooter.waitUntilTargetVelocity(),
        new ScoreFeed()
      )
    );
  }
}
