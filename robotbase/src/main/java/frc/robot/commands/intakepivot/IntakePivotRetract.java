package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakePivotRetract extends ParallelDeadlineGroup {

  public IntakePivotRetract() {
    super(
      new WaitUntilIntakePivotStall(),
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.RETRACT_SPEED),
      Commands.runEnd(
        () -> SmartDashboard.putBoolean("Deploying", true),
        () -> SmartDashboard.putBoolean("Deploying", false)
      )
    );
  }
}
