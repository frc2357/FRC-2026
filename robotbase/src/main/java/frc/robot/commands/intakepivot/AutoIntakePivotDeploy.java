package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class AutoIntakePivotDeploy extends SequentialCommandGroup {

  public AutoIntakePivotDeploy() {
    super(
      new ParallelRaceGroup(
        new WaitUntilIntakePivotDeploy(),
        new WaitCommand(Constants.INTAKE_PIVOT.INTAKE_MAXIMUM_STALL_TIME),
        Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED)
      )
    );
  }
}
