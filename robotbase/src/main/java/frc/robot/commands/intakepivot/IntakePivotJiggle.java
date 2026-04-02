package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;

public class IntakePivotJiggle extends ParallelCommandGroup {

  public IntakePivotJiggle() {
    super(
      new IntakeRunnerSetSpeed(Constants.INTAKE_RUNNER.INTAKE_JIGGLING_SPEED),
      new RepeatCommand(
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.JIGGLE_UP_SPEED),
            new WaitUntilIntakePivotStall(),
            new WaitUntilIntakePivotAbovePosition(
              Constants.INTAKE_PIVOT.INTAKE_JIGGLE_UP_ENCODER_ROTATIONS
            )
            // new WaitCommand(Constants.INTAKE_PIVOT.JIGGLE_UP_TIME)
          ),
          new ParallelRaceGroup(
            Robot.intakePivot.setSpeed(
              Constants.INTAKE_PIVOT.JIGGLE_DOWN_SPEED
            ),
            new WaitUntilIntakePivotStall()
            //new WaitUntilIntakePivotStallAndPosition()
          )
        )
      )
    );
  }
}
