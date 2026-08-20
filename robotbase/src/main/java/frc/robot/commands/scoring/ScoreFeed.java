package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederSetVelocity;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.kicker.KickerSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class ScoreFeed extends ParallelCommandGroup {

  public ScoreFeed() {
    super(
      new FeederSetVelocity(Constants.FEEDER.FEED_SPEED),
      new KickerSetSpeed(Constants.KICKER.KICK_SPEED),
      new TunnelSetSpeed(Constants.TUNNEL.TUNNEL_SPEED),
      new FloorSetSpeed(Constants.FLOOR.FLOOR_SPEED)
    );
  }
}
