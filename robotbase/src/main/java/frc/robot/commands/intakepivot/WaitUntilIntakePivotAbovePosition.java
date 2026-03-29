package frc.robot.commands.intakepivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class WaitUntilIntakePivotAbovePosition extends WaitUntilCommand {

  public WaitUntilIntakePivotAbovePosition(Angle targetAngle) {
    super(Robot.intakePivot.isAbovePositionTrigger(targetAngle));
  }
}
