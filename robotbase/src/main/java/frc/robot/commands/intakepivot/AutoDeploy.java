package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoDeploy extends SequentialCommandGroup {

  /**
   * For use with autos, really makes sure the intake is down
   */
  public AutoDeploy() {
    super(
      new IntakePivotDeploy(),
      new IntakePivotDeploy(),
      new IntakePivotDeploy()
    );
  }
}
