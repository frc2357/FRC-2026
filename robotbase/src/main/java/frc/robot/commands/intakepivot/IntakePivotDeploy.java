package frc.robot.commands.intakePivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakePivotDeploy extends ParallelDeadlineGroup {

  public IntakePivotDeploy() {
    super(
      new WaitUntilIntakePivotStall().finallyDo((boolean interrupted) ->
        // Only say we have deployed intake if this command finishes
        Robot.intakePivot.setDeployed(!interrupted)
      ),
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED)
    );
  }
}
