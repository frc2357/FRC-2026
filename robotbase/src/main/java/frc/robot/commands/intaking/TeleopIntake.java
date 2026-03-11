package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE_RUNNER;
import frc.robot.commands.drive.SetDriveModifiers;
import frc.robot.commands.intake_runner.IntakeRunnerSetSpeed;
import frc.robot.commands.intakepivot.IntakeDeploy;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new SetDriveModifiers(),
      new SequentialCommandGroup(
        new IntakeDeploy(),
        new IntakeRunnerSetSpeed(INTAKE_RUNNER.TELEOP_INTAKING_SPEED)
      )
    );
  }
}
