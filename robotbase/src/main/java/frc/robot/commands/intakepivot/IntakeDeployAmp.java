package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeDeployAmp extends ParallelCommandGroup {

  public IntakeDeployAmp() {
    super(
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED),
      new SequentialCommandGroup(
        new WaitCommand(Constants.INTAKE_PIVOT.DEPLOY_NORMALIZE_DELAY),
        new WaitUntilCommand(() ->
          Robot.intakePivot
            .getStatorCurrent()
            .gte(Constants.INTAKE_PIVOT.DEPLOY_AMP_THRESHOLD)
        )
      )
    );
  }
}
