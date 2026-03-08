package frc.robot.commands.intakePivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class IntakePivotJiggle extends RepeatCommand {

  public IntakePivotJiggle(Angle topAngle, Angle bottomAngle) {
    super(
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitUntilCommand(() ->
            Robot.intakePivot.getAngle().isNear(topAngle, .1)
          ),
          Robot.intakePivot.setAngle(topAngle)
        ),
        new ParallelRaceGroup(
          Robot.intakePivot.setAngle(bottomAngle),
          new WaitUntilCommand(() ->
            Robot.intakePivot.getAngle().isNear(bottomAngle, .1)
          ),
          Robot.intakePivot.setAngle(bottomAngle)
        )
      )
    );
  }
}
