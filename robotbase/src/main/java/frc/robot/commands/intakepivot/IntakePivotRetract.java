package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakePivotRetract extends ParallelDeadlineGroup {

  public IntakePivotRetract() {
    super(
      new WaitUntilIntakePivotStall().finallyDo(() ->
        Robot.intakePivot.setDeployed(false)
      ),
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.RETRACT_SPEED)
    );
  }
}
