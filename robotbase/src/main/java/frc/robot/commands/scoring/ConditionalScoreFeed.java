package frc.robot.commands.scoring;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

public class ConditionalScoreFeed extends Command {

  private Trigger m_condition;

  public ConditionalScoreFeed(Trigger condition) {
    m_condition = condition;
  }

  @Override
  public void execute() {
    if (m_condition.getAsBoolean()) {
      Robot.feeder.setVelocitySetpoint(
        RotationsPerSecond.of(
          SmartDashboard.getNumber(
            "feed speed",
            Constants.FEEDER.FEED_SPEED.in(RotationsPerSecond)
          )
        )
      );
      Robot.kicker.setSpeed(
        Value.of(
          SmartDashboard.getNumber(
            "kicker speed",
            Constants.KICKER.KICK_SPEED.in(Value)
          )
        )
      );
      Robot.tunnel.setSpeed(
        Value.of(
          SmartDashboard.getNumber(
            "tunnel speed",
            Constants.TUNNEL.TUNNEL_SPEED.in(Value)
          )
        )
      );
      Robot.floor.setSpeed(
        Value.of(
          SmartDashboard.getNumber(
            "floor speed",
            Constants.FLOOR.FLOOR_SPEED.in(Value)
          )
        )
      );
    } else {
      stopAllFeeding();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    stopAllFeeding();
  }

  private void stopAllFeeding() {
    Robot.feeder.stopMotor();
    Robot.kicker.stop();
    Robot.tunnel.stop();
    Robot.floor.stop();
  }
}
