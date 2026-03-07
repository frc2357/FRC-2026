package frc.robot.commands.controller;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CONTROLLER;
import frc.robot.Robot;

public class RumbleCoDriverController extends Command {

  Timer timer = new Timer();

  @Override
  public void initialize() {
    Robot.driverControls.setRumble(CONTROLLER.CODRIVER_RUMBLE_INTENSITY);
    timer.restart();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(CONTROLLER.CODRIVER_RUMBLE_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    Robot.driverControls.setRumble(0);
  }
}
