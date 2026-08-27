package frc.robot.commands.intakepivot.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intakepivot.IntakePivotSetSpeed;
import frc.robot.commands.intakepivot.waits.WaitUntilIntakePivotDeploy;

public class AutoIntakePivotDeploy extends SequentialCommandGroup {

  public AutoIntakePivotDeploy() {
    super(
      new ParallelRaceGroup(
        new WaitUntilIntakePivotDeploy(),
        new WaitCommand(Constants.INTAKE_PIVOT.INTAKE_MAXIMUM_STALL_TIME),
        new IntakePivotSetSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED)
      )
    );
  }
}
