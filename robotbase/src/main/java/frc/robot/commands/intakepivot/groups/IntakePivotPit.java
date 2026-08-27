package frc.robot.commands.intakepivot.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.intakepivot.IntakePivotSetSpeed;
import frc.robot.commands.intakepivot.waits.WaitUntilIntakePivotStall;

public class IntakePivotPit extends SequentialCommandGroup {

  public IntakePivotPit() {
    super(
      new RepeatCommand(
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new IntakePivotSetSpeed(Constants.INTAKE_PIVOT.PIT_UP_SPEED),
            new WaitUntilIntakePivotStall(),
            new WaitCommand(Constants.INTAKE_PIVOT.PIT_UP_TIME)
          ),
          new ParallelRaceGroup(
            new IntakePivotSetSpeed(Constants.INTAKE_PIVOT.JIGGLE_DOWN_SPEED),
            new WaitUntilIntakePivotStall()
          )
        )
      )
    );
  }
}
