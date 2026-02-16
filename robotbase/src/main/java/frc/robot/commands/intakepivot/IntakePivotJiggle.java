package frc.robot.commands.intakepivot;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeSetSpeed;

public class IntakePivotJiggle extends ParallelCommandGroup {

  public IntakePivotJiggle() {
    super(
      new IntakePivotSetSpeed(
        Value.of(SmartDashboard.getNumber("IntakePivotSpeed", .5))
      ),
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(SmartDashboard.getNumber("IntakeTimeOn", .5)),
          new IntakeSetSpeed(
            Value.of(SmartDashboard.getNumber("IntakeSpeed", .5))
          )
        ),
        new WaitCommand(SmartDashboard.getNumber("IntakeTimeOff", .5))
      )
    );
  }
}
