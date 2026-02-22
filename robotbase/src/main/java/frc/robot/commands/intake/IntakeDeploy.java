package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.INTAKE_PIVOT;
import frc.robot.Robot;

public class IntakeDeploy extends Command {

  public IntakeDeploy() {
    addRequirements(Robot.intakePivot);
  }

  @Override
  public void initialize() {
    Robot.intakePivot.setAngleSetpoint(INTAKE_PIVOT.DEPLOYED_ANGLE);
  }

  @Override
  public boolean isFinished() {
    return Robot.intakePivot
      .getAngle()
      .isNear(INTAKE_PIVOT.DEPLOYED_ANGLE, INTAKE_PIVOT.DEPLOYED_TOLERANCE);
  }
}
