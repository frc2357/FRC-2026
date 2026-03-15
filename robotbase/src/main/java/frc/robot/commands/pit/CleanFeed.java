package frc.robot.commands.pit;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class CleanFeed extends ParallelCommandGroup {

  public CleanFeed() {
    super(
      new FeederSetSpeed(() ->
        Value.of(
          SmartDashboard.getNumber(
            "feed speed",
            Constants.FEEDER.SLOW_FEED_SPEED.in(Value)
          )
        )
      ),
      new TunnelSetSpeed(
        Value.of(
          SmartDashboard.getNumber(
            "tunnel speed",
            Constants.TUNNEL.SLOW_TUNNEL_SPEED.in(Value)
          )
        )
      ),
      new FloorSetSpeed(
        Value.of(
          SmartDashboard.getNumber(
            "floor speed",
            Constants.FLOOR.SLOW_FLOOR_SPEED.in(Value)
          )
        )
      )
    );
  }
}
