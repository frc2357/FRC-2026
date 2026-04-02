package frc.robot.commands.scoring;

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
      SmartDashboard.putBoolean("feeding", true);
      Robot.feeder.setVelocitySetpoint(Constants.FEEDER.FEED_SPEED);
      Robot.kicker.setSpeed(Constants.KICKER.KICK_SPEED);
      Robot.tunnel.setSpeed(Constants.TUNNEL.TUNNEL_SPEED);
      Robot.floor.setSpeed(Constants.FLOOR.FLOOR_SPEED);
    } else {
      stopAllFeeding();
      SmartDashboard.putBoolean("feeding", false);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("feeding", false);
    stopAllFeeding();
  }

  private void stopAllFeeding() {
    Robot.feeder.stopMotor();
    Robot.kicker.stop();
    Robot.tunnel.stop();
    Robot.floor.stop();
  }
}
