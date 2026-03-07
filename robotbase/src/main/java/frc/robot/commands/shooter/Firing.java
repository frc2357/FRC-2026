package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FEEDER;
import frc.robot.Constants.FLOOR;
import frc.robot.Constants.TUNNEL;
import frc.robot.Robot;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.floor.FloorSetSpeed;
import frc.robot.commands.tunnel.TunnelSetSpeed;

public class Firing extends ParallelCommandGroup {

  public Firing(AngularVelocity rpm) {
    super(
      Robot.shooter.setVelocity(rpm),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> Robot.shooter.getVelocity().isNear(rpm, 5)), //TODO: replace with WaitUntilTargetVelocity
        new ParallelCommandGroup(
          new FeederSetSpeed(FEEDER.FEED_SPEED),
          new FloorSetSpeed(FLOOR.FLOOR_SPEED),
          new TunnelSetSpeed(TUNNEL.TUNNEL_SPEED)
        )
      )
    );
  }
}
