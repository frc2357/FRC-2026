package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class AutoDeploy extends SequentialCommandGroup {

  /**
   * For use with autos, really makes sure the intake is down
   */
  public AutoDeploy() {
    super(
      new InstantCommand(() -> Robot.intakePivot.zeroMotorEncoder()),
      new AutoIntakePivotDeploy()
    );
  }
}
