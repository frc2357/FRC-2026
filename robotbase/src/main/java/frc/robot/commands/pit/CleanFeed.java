package frc.robot.commands.pit;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;
import frc.robot.commands.kicker.KickerSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class CleanFeed extends ParallelCommandGroup {

  public CleanFeed() {
    super(
      Robot.feeder.setSpeed(
        Value.of(Constants.FEEDER.SLOW_FEED_SPEED.in(Value))
      ),
      new TunnelSetSpeed(
        Value.of(Constants.TUNNEL.SLOW_TUNNEL_SPEED.in(Value))
      ),
      new FloorSetSpeed(Value.of(Constants.FLOOR.SLOW_FLOOR_SPEED.in(Value))),
      new IntakeRunnerSetSpeed(
        Value.of(Constants.INTAKE_RUNNER.CLEAN_SPEED.in(Value))
      ),
      new KickerSetSpeed(Value.of(Constants.KICKER.CLEAN_SPEED.in(Value)))
    );
  }
}
