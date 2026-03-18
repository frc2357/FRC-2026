package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakePivotPit extends SequentialCommandGroup {

  public IntakePivotPit() {
    super(
      new InstantCommand(() -> Robot.intakePivot.setDeployed(false)),
      new RepeatCommand(
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.JIGGLE_UP_SPEED),
            new WaitUntilIntakePivotStall(),
            new WaitCommand(Constants.INTAKE_PIVOT.JIGGLE_UP_TIME)
          ),
          new ParallelRaceGroup(
            Robot.intakePivot.setSpeed(
              Constants.INTAKE_PIVOT.JIGGLE_DOWN_SPEED
            ),
            new WaitUntilIntakePivotStall()
          )
        )
      )
    );
  }
}
