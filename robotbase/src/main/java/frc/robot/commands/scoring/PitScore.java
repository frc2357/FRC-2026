package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import java.util.function.Supplier;

public class PitScore extends ParallelCommandGroup {

  public PitScore(AngularVelocity shooterVelocity, Angle hoodAngle) {
    this(() -> shooterVelocity, () -> hoodAngle);
  }

  public PitScore(
    Supplier<AngularVelocity> shooterVelocity,
    Supplier<Angle> hoodAngle
  ) {
    super();
    addCommands(
      Robot.shooter.setVelocity(shooterVelocity),
      Robot.hood.setAngle(hoodAngle),
      new SequentialCommandGroup(new ScoreFeed())
    );
  }
}
