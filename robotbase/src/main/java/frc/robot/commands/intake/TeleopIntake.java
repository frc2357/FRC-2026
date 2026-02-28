package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.INTAKE;
import frc.robot.commands.drive.SetDriveModifiers;

public class TeleopIntake extends ParallelCommandGroup {

  public TeleopIntake() {
    super(
      new SetDriveModifiers(),
      new SequentialCommandGroup(
        new IntakeDeploy(),
        new IntakeSetSpeed(INTAKE.TELEOP_INTAKING_SPEED)
      )
    );
  }
}
