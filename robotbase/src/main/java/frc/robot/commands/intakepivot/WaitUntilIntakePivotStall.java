package frc.robot.commands.intakepivot;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

public class WaitUntilIntakePivotStall extends WaitUntilCommand {

  public WaitUntilIntakePivotStall() {
    super(Robot.intakePivot.isIntakeCurrentStallingTrigger());
  }
}
