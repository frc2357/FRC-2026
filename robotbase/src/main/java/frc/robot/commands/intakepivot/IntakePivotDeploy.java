package frc.robot.commands.intakepivot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakePivotDeploy extends ParallelDeadlineGroup {

  public IntakePivotDeploy() {
    super(
      new WaitUntilCommand(
        new Trigger(() ->
          Robot.intakePivot
            .getStatorCurrent()
            .gte(Constants.INTAKE_PIVOT.AMP_STALL_THRESHOLD)
        ).debounce(Constants.INTAKE_PIVOT.TIME_TO_STALL.in(Seconds))
      ),
      Robot.intakePivot.setSpeed(Constants.INTAKE_PIVOT.DEPLOY_SPEED),
      Commands.runEnd(
        () -> SmartDashboard.putBoolean("Deploying", true),
        () -> SmartDashboard.putBoolean("Deploying", false)
      )
    );
  }
}
