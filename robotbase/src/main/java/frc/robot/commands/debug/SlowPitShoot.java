package frc.robot.commands.debug;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class SlowPitShoot extends ParallelCommandGroup {

  public SlowPitShoot() {
    super(
      new FeederSetSpeed(Constants.FEEDER.FEED_SPEED),
      new TunnelSetSpeed(Constants.TUNNEL.TUNNEL_SPEED),
      new FloorSetSpeed(Constants.FLOOR.FLOOR_SPEED),
      Robot.shooter.setSpeed(Constants.SHOOTER.SLOW_SPEED)
    );
  }
}
