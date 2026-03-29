package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drive.InitSwerve;

public class InitRobotCommand extends SequentialCommandGroup {

  public InitRobotCommand() {
    super(
      Robot.intakePivot.zeroMotorEncoder(),
      new ParallelCommandGroup(new SetAlliance(), new SetMatchType()),
      new InitSwerve()
    );
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
