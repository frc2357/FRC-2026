package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class TunnelFeedReverse extends ParallelCommandGroup {

  public TunnelFeedReverse() {
    super(
      Robot.feeder.setSpeed(Constants.FEEDER.REVERSE_FEED_SPEED),
      new TunnelSetSpeed(Constants.TUNNEL.REVERSE_TUNNEL_SPEED)
    );
  }
}
