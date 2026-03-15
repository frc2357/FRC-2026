package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TestSwerve extends SequentialCommandGroup {

  public TestSwerve() {
    super(
      new InstantCommand(() -> new ResetPerspective()),
      new ParallelDeadlineGroup(new WaitCommand(1), new VelocityDrive()),
      new ParallelDeadlineGroup(new WaitCommand(1), new VelocityDrive()),
      new ParallelDeadlineGroup(new WaitCommand(1), new VelocityDrive())
    );
  }
}
