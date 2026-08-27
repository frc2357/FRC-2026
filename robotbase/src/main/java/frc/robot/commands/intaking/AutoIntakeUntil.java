package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.INTAKE_PIVOT;
import frc.robot.Constants.INTAKE_RUNNER;
import frc.robot.commands.intakepivot.IntakePivotSetSpeed;
import frc.robot.commands.intakepivot.groups.IntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;
import java.util.function.BooleanSupplier;

public class AutoIntakeUntil extends ParallelDeadlineGroup {

  public AutoIntakeUntil(BooleanSupplier untilCondition) {
    super(
      new WaitUntilCommand(untilCondition),
      new SequentialCommandGroup(
        new IntakePivotDeploy(),
        new IntakePivotSetSpeed(INTAKE_PIVOT.HOLD_DOWN_SPEED)
      ),
      new IntakeRunnerSetSpeed(INTAKE_RUNNER.TELEOP_INTAKING_SPEED)
    );
  }
}
