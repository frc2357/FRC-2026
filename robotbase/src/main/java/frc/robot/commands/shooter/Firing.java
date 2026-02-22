package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FEEDER;
import frc.robot.Constants.SPINDEXER;
import frc.robot.Robot;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.spindexer.SpindexerSetSpeed;

public class Firing extends ParallelCommandGroup {

  public Firing(AngularVelocity rpm) {
    super(
      Robot.shooter.setVelocity(rpm),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> Robot.shooter.getVelocity().isNear(rpm, 5)), //TODO: add velocity tolerance once it's merged from tuning
        new ParallelCommandGroup(
          new FeederSetSpeed(FEEDER.FEED_SPEED),
          new SpindexerSetSpeed(SPINDEXER.SPINDEXER_SPEED)
        )
      )
    );
  }
}
