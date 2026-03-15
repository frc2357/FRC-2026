package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.INTAKE_RUNNER;
import frc.robot.commands.drive.SetActionDriveModifiers;
import frc.robot.commands.intakepivot.IntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new SetActionDriveModifiers(),
      new IntakePivotDeploy(),
      new IntakeRunnerSetSpeed(INTAKE_RUNNER.TELEOP_INTAKING_SPEED)
    );
  }
}
