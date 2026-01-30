package frc.robot.commands.outtake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class OuttakeStop extends Command {

  public OuttakeStop() {
    addRequirements(Robot.outtake);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }
}
