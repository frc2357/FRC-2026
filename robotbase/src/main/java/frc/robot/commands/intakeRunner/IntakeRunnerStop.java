package frc.robot.commands.intakerunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeRunnerStop extends Command {

  public IntakeRunnerStop() {
    addRequirements(Robot.intake);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intake.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
