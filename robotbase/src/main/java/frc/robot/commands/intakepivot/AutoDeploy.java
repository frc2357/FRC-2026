package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoDeploy extends SequentialCommandGroup {

  /**
   * For use with autos, really makes sure the intake is down
   */
  public AutoDeploy() {
    super(
      new IntakePivotDeploy(),
      new WaitCommand(0.25),
      new IntakePivotDeploy(),
      new WaitCommand(0.25),
      new IntakePivotDeploy()
    );
  }
}
