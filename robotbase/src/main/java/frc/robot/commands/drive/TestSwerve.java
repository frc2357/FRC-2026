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
      new SequentialCommandGroup(
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new DefaultDrive(
            () -> Value.of(0),
            () -> Value.of(0.5),
            () -> Value.of(0)
          )
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(1),
          new DefaultDrive(
            () -> Value.of(0),
            () -> Value.of(-0.5),
            () -> Value.of(0)
          ),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new DefaultDrive(
              () -> Value.of(0),
              () -> Value.of(0),
              () -> Value.of(.5)
            )
          )
        )
      )
    );
  }
}
