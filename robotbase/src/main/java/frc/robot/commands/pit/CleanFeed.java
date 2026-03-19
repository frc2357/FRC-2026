package frc.robot.commands.pit;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SHOOTER;
import frc.robot.Robot;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;
import frc.robot.commands.kicker.KickerSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class CleanFeed extends ParallelCommandGroup {

  public CleanFeed() {
    super(
      Robot.feeder.setSpeed(Constants.FEEDER.SLOW_FEED_SPEED),
      new TunnelSetSpeed(Constants.TUNNEL.SLOW_TUNNEL_SPEED),
      new FloorSetSpeed(Constants.FLOOR.SLOW_FLOOR_SPEED),
      new IntakeRunnerSetSpeed(Constants.INTAKE_RUNNER.CLEAN_SPEED),
      new KickerSetSpeed(Constants.KICKER.CLEAN_SPEED),
      Robot.shooter.setSpeed(SHOOTER.CLEAN_SPEED)
    );
  }
}
