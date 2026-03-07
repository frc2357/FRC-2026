package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class TunnelFeedReverse extends ParallelCommandGroup {

  public TunnelFeedReverse() {
    super(
      new FeederSetSpeed(Constants.FEEDER.REVERSE_FEED_SPEED),
      new TunnelSetSpeed(Constants.TUNNEL.REVERSE_TUNNEL_SPEED)
    );
  }
}
