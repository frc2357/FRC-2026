package frc.robot.commands.pit;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.CrossWheels;
import frc.robot.commands.drive.ResetPerspective;
import frc.robot.commands.drive.VelocityDrive;

public class TestSwerve extends SequentialCommandGroup {

  public TestSwerve() {
    super(
      new InstantCommand(() -> new ResetPerspective()),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new VelocityDrive(Percent.of(50), Percent.of(0), Percent.of(0))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new VelocityDrive(Percent.of(0), Percent.of(50), Percent.of(0))
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new VelocityDrive(Percent.of(0), Percent.of(0), Percent.of(50))
      ),
      new CrossWheels()
    );
  }
}
