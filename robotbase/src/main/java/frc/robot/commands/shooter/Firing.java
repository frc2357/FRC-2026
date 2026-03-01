package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FEEDER;
import frc.robot.Constants.FLOOR;
import frc.robot.Robot;
import frc.robot.commands.feeder.FeederSetSpeed;
import frc.robot.commands.floor.FloorSetSpeed;

public class Firing extends ParallelCommandGroup {

  public Firing(AngularVelocity rpm) {
    super(
      Robot.shooter.setVelocity(() ->
        RotationsPerSecond.of(SmartDashboard.getNumber("Shooter RPS", 0))
      ),
      new SequentialCommandGroup(
        new WaitUntilCommand(() -> Robot.shooter.getVelocity().isNear(rpm, 5)), //TODO: replace with WaitUntilTargetVelocity
        new ParallelCommandGroup(
          new FeederSetSpeed(FEEDER.FEED_SPEED),
          new FloorSetSpeed(FLOOR.FLOOR_SPEED)
        )
      )
    );
  }
}
