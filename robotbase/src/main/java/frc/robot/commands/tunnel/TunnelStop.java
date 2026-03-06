package frc.robot.commands.tunnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TunnelStop extends Command {

  public TunnelStop() {
    addRequirements(Robot.floor);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.tunnel.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
