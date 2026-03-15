package frc.robot.commands.intaking;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE_RUNNER;
import frc.robot.Robot;
import frc.robot.commands.drive.SetIntakeDriveModifiers;
import frc.robot.commands.intakePivot.IntakePivotDeploy;
import frc.robot.commands.intakerunner.IntakeRunnerSetSpeed;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new SetIntakeDriveModifiers(),
      new SequentialCommandGroup(
        // Conditionally deploy intake to reduce power draw
        new ConditionalCommand(
          new InstantCommand(),
          new IntakePivotDeploy(),
          () -> Robot.intakePivot.hasDeployed()
        ),
        new IntakeRunnerSetSpeed(INTAKE_RUNNER.TELEOP_INTAKING_SPEED)
      )
    );
  }
}
