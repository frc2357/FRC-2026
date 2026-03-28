package frc.robot.commands.scoring.auto;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.scoring.ScoreFeed;
import java.util.function.Supplier;

public class AutoScore extends ParallelCommandGroup {

  public AutoScore(AngularVelocity shooterVelocity, Angle hoodAngle) {
    this(() -> shooterVelocity, () -> hoodAngle);
  }

  public AutoScore(
    Supplier<AngularVelocity> shooterVelocity,
    Supplier<Angle> hoodAngle
  ) {
    super(
      Robot.shooter.setVelocity(shooterVelocity),
      Robot.hood.setAngle(hoodAngle),
      new SequentialCommandGroup(
        new WaitUntilCommand(Robot.shooter.isAtContinuousTargetVelocity()),
        new ScoreFeed()
      )
    );
  }
}
