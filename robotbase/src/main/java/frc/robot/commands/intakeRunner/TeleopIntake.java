package frc.robot.commands.intakeRunner;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKERUNNER;
import frc.robot.commands.drive.SetDriveModifiers;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new SetDriveModifiers(),
      new SequentialCommandGroup(
        new IntakeDeploy(),
        new IntakeRunnerSetSpeed(INTAKERUNNER.TELEOP_INTAKING_SPEED)
      )
    );
  }
}
