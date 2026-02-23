package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE_PIVOT;
import frc.robot.Robot;

public class DefaultIntakePivot extends Command {

  public DefaultIntakePivot() {
    addRequirements(Robot.intakePivot);
  }

  @Override
  public void initialize() {
    Robot.intakePivot.setSpeed(INTAKE_PIVOT.HOLD_DOWN_SPEED);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.intakePivot.stopMotor();
  }
}
