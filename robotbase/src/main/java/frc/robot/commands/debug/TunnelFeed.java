package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class TunnelFeed extends ParallelCommandGroup {

  public TunnelFeed() {
    super(
      Robot.feeder.setSpeed(Constants.FEEDER.FEED_SPEED_PERCENT),
      new TunnelSetSpeed(Constants.TUNNEL.TUNNEL_SPEED)
    );
  }
}
