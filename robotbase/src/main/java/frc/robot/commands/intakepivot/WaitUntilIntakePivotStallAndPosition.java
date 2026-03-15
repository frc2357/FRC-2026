package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class WaitUntilIntakePivotStallAndPosition extends WaitUntilCommand {

  public WaitUntilIntakePivotStallAndPosition() {
    super(
      Robot.intakePivot
        .isInDeployedPositionTrigger()
        .and(Robot.intakePivot.isIntakeStallingTrigger())
    );
  }
}
