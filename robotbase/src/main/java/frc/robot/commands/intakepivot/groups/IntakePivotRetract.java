package frc.robot.commands.intakepivot.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.commands.intakepivot.IntakePivotSetSpeed;
import frc.robot.commands.intakepivot.waits.WaitUntilIntakePivotStall;

public class IntakePivotRetract extends ParallelDeadlineGroup {

  public IntakePivotRetract() {
    super(
      new WaitUntilIntakePivotStall(),
      new IntakePivotSetSpeed(Constants.INTAKE_PIVOT.RETRACT_SPEED)
    );
  }
}
