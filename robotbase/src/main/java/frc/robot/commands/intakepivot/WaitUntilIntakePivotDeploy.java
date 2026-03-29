package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class WaitUntilIntakePivotDeploy extends WaitUntilCommand {

  public WaitUntilIntakePivotDeploy() {
    super(
      Robot.intakePivot
        .isInDeployedPositionTrigger()
        .and(Robot.intakePivot.isIntakeVelocityStallingTrigger())
      // .and(Robot.intakePivot.isIntakeCurrentStallingTrigger())
    );
  }
}
