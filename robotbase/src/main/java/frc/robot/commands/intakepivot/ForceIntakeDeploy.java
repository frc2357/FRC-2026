package frc.robot.commands.intakePivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

// To be used for panic controls only
public class ForceIntakeDeploy extends SequentialCommandGroup {

  public ForceIntakeDeploy() {
    super(
      new InstantCommand(() -> Robot.intakePivot.setDeployed(false)),
      new IntakePivotDeploy()
    );
  }
}
