package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakePivotDeploy extends ParallelDeadlineGroup {

  public IntakePivotDeploy() {
    super(
      new WaitUntilIntakePivotStall(),
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED)
    );
  }
}
