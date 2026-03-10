package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class ScoreFeed extends ParallelCommandGroup {

  public ScoreFeed() {
    super(
      new FeederSetSpeed(Constants.FEEDER.FEED_SPEED),
      new TunnelSetSpeed(Constants.TUNNEL.TUNNEL_SPEED),
      new FloorSetSpeed(Constants.FLOOR.FLOOR_SPEED)
    );
  }
}
