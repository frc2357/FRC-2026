package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE_PIVOT;
import frc.robot.Constants.INTAKE_RUNNER;
import frc.robot.Robot;
import frc.robot.commands.intakepivot.IntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new SequentialCommandGroup(
        new IntakePivotDeploy(),
        Robot.intakePivot.setSpeed(INTAKE_PIVOT.HOLD_DOWN_SPEED)
      ),
      new IntakeRunnerSetSpeed(INTAKE_RUNNER.TELEOP_INTAKING_SPEED)
    );
  }
}
