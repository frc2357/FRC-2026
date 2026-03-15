package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoIntakePivotDeploy extends ParallelDeadlineGroup {

  public AutoIntakePivotDeploy() {
    super(
      new WaitUntilIntakePivotStallAndPosition(),
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED)
    );
  }
}
