package frc.robot.commands.intakepivot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

public class WaitUntilIntakePivotStall extends WaitUntilCommand {

  public WaitUntilIntakePivotStall() {
    super(
      new Trigger(() ->
        Robot.intakePivot
          .getStatorCurrent()
          .gte(Constants.INTAKE_PIVOT.AMP_STALL_THRESHOLD)
      ).debounce(Constants.INTAKE_PIVOT.TIME_TO_STALL.in(Seconds))
    );
  }
}
